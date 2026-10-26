"""Static MJCF preview using MuJoCo's model loader and renderer, no simulation loop."""
from dataclasses import dataclass
from pathlib import Path

@dataclass
class PreviewModel:
    model: object
    data: object
    center: object
    dimensions: tuple
    mass: float
    wheel_count: int
    wheel_diameter: float | None

def load_preview_model(asset):
    import mujoco
    import numpy as np
    asset=Path(asset).resolve()
    if not asset.is_file():raise FileNotFoundError(asset)
    if asset.suffix.lower()!='.xml':raise ValueError('preview requires MJCF XML')
    model=mujoco.MjModel.from_xml_path(str(asset));data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    if model.ngeom==0:raise ValueError('robot has no visible geometry')
    minima=[];maxima=[];wheels=[]
    for i in range(model.ngeom):
        rotation=data.geom_xmat[i].reshape(3,3)
        center=data.geom_xpos[i]+rotation@model.geom_aabb[i,:3]
        half=np.abs(rotation)@model.geom_aabb[i,3:]
        minima.append(center-half);maxima.append(center+half)
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[i])) or ''
        if model.geom_group[i]!=2 and 'wheel' in name and model.geom_type[i]==mujoco.mjtGeom.mjGEOM_CYLINDER:wheels.append(float(model.geom_size[i,0])*2)
    lower=np.min(minima,axis=0);upper=np.max(maxima,axis=0)
    return PreviewModel(model,data,(lower+upper)/2,tuple(float(v) for v in upper-lower),float(model.body_mass.sum()),len(wheels),max(wheels) if wheels else None)

def show_preview(asset,label='Robot',screenshot=None,smoke=False,watch=False):
    import glfw,mujoco,numpy as np,signal
    preview=load_preview_model(asset);model=preview.model;data=preview.data
    model.vis.headlight.active=1;model.vis.headlight.ambient[:]=.5;model.vis.headlight.diffuse[:]=.8;model.vis.headlight.specular[:]=.2
    if not glfw.init():raise RuntimeError('3D preview could not initialize GLFW')
    window=glfw.create_window(960,720,f'ステップコイン・アリーナ 3D — {label}',None,None)
    if not window:glfw.terminate();raise RuntimeError('3D preview could not create an OpenGL window')
    context=None
    try:
        glfw.make_context_current(window);glfw.swap_interval(1)
        scene=mujoco.MjvScene(model,maxgeom=max(1000,model.ngeom*2));context=mujoco.MjrContext(model,mujoco.mjtFontScale.mjFONTSCALE_150)
        option=mujoco.MjvOption();camera=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(model,camera)
        def reset_camera():
            camera.lookat[:]=preview.center;camera.distance=max(preview.dimensions)*2.2;camera.azimuth=135;camera.elevation=-24
        reset_camera();dirty=True;last=None
        def redraw():
            nonlocal dirty
            dirty=True
        def cursor(win,x,y):
            nonlocal last
            if last is None:last=(x,y);return
            dx=x-last[0];dy=y-last[1];last=(x,y)
            left=glfw.get_mouse_button(win,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS
            right=glfw.get_mouse_button(win,glfw.MOUSE_BUTTON_RIGHT)==glfw.PRESS
            if not (left or right):return
            shift=glfw.get_key(win,glfw.KEY_LEFT_SHIFT)==glfw.PRESS or glfw.get_key(win,glfw.KEY_RIGHT_SHIFT)==glfw.PRESS
            action=mujoco.mjtMouse.mjMOUSE_MOVE_H if right or shift else mujoco.mjtMouse.mjMOUSE_ROTATE_H
            height=max(1,glfw.get_window_size(win)[1]);mujoco.mjv_moveCamera(model,action,dx/height,dy/height,camera);redraw()
        def scroll(win,x,y):
            mujoco.mjv_moveCamera(model,mujoco.mjtMouse.mjMOUSE_ZOOM,0,-.05*y,camera);redraw()
        def key(win,key,scancode,action,mods):
            if action!=glfw.PRESS:return
            if key in (glfw.KEY_ESCAPE,):glfw.set_window_should_close(win,True)
            elif key==glfw.KEY_R:reset_camera();redraw()
            elif key==glfw.KEY_V:option.geomgroup[2]=not option.geomgroup[2];redraw()
        glfw.set_cursor_pos_callback(window,cursor);glfw.set_scroll_callback(window,scroll);glfw.set_key_callback(window,key)
        glfw.set_framebuffer_size_callback(window,lambda *args:redraw());glfw.set_window_refresh_callback(window,lambda *args:redraw())
        signal.signal(signal.SIGTERM,lambda *args:glfw.set_window_should_close(window,True))
        if smoke:
            before=float(camera.distance);scroll(window,0,1);assert camera.distance!=before
            glfw.set_cursor_pos(window,400,300);cursor(window,400,300)
            azimuth=float(camera.azimuth);mujoco.mjv_moveCamera(model,mujoco.mjtMouse.mjMOUSE_ROTATE_H,.1,.05,camera)
            assert camera.azimuth!=azimuth
            lookat=camera.lookat.copy();mujoco.mjv_moveCamera(model,mujoco.mjtMouse.mjMOUSE_MOVE_H,.1,.05,camera)
            assert not np.allclose(camera.lookat,lookat)
            key(window,glfw.KEY_R,0,glfw.PRESS,0);assert np.allclose(camera.lookat,preview.center)
            detailed=bool(option.geomgroup[2]);key(window,glfw.KEY_V,0,glfw.PRESS,0);assert bool(option.geomgroup[2])!=detailed
            key(window,glfw.KEY_V,0,glfw.PRESS,0);assert bool(option.geomgroup[2])==detailed
            assert data.time==0
        captured=False;watch_signature=None;reload_count=0
        while not glfw.window_should_close(window):
            if watch:
                import json
                metadata=Path(asset).parent/'view.json'
                signature=tuple((p.stat().st_mtime_ns,p.stat().st_size) if p.exists() else None for p in (Path(asset),metadata))
                if signature!=watch_signature:
                    watch_signature=signature
                    try:
                        info=json.loads(metadata.read_text()) if metadata.exists() else {}
                        label=info.get('label',label)
                        if info.get('error'):
                            glfw.set_window_title(window,'3D — '+label+' / previous geometry retained')
                        else:
                            updated=load_preview_model(asset)
                            new_scene=mujoco.MjvScene(updated.model,maxgeom=max(1000,updated.model.ngeom*2))
                            new_context=mujoco.MjrContext(updated.model,mujoco.mjtFontScale.mjFONTSCALE_150)
                            context.free();context=new_context;scene=new_scene
                            preview=updated;model=preview.model;data=preview.data;reload_count+=1;captured=False
                            model.vis.headlight.active=1;model.vis.headlight.ambient[:]=.5;model.vis.headlight.diffuse[:]=.8;model.vis.headlight.specular[:]=.2
                            glfw.set_window_title(window,'3D — '+label)
                        dirty=True
                    except Exception as error:
                        glfw.set_window_title(window,'3D — Reload failed / previous geometry retained')
                        print('Preview reload failed:',str(error),flush=True)
            if dirty:
                width,height=glfw.get_framebuffer_size(window);viewport=mujoco.MjrRect(0,0,width,height)
                mujoco.mjv_updateScene(model,data,option,None,camera,mujoco.mjtCatBit.mjCAT_ALL,scene)
                # A render-only ground reference: it never enters the physical model.
                floor=scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(floor,mujoco.mjtGeom.mjGEOM_PLANE,np.array([1.5,1.5,.01]),np.array([preview.center[0],preview.center[1],preview.center[2]-preview.dimensions[2]/2-.01]),np.eye(3).reshape(-1),np.array([.32,.36,.42,1.]))
                scene.ngeom+=1
                mujoco.mjr_render(viewport,scene,context)
                dimensions=' x '.join(f'{v:.3f}' for v in preview.dimensions)
                specs=f'Size XYZ: {dimensions} m\nPhysics mass: {preview.mass:.2f} kg\nWheels: {preview.wheel_count}'
                if preview.wheel_diameter is not None:specs+=f'  Diameter: {preview.wheel_diameter:.3f} m'
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,viewport,'Static robot preview'+(' | VISUAL CONCEPT' if model.ngeom>100 else '')+'\n'+specs,'',context)
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,viewport,'Drag: rotate | Right drag / Shift drag: pan\nWheel: zoom | R: reset view | V: details on/off | Esc: close','',context)
                if screenshot and not captured:
                    from PIL import Image
                    rgb=np.empty((height,width,3),dtype=np.uint8);mujoco.mjr_readPixels(rgb,None,viewport,context)
                    Image.fromarray(np.flipud(rgb)).save(screenshot);captured=True
                glfw.swap_buffers(window);dirty=False
                if smoke:glfw.set_window_should_close(window,True)
            else:glfw.wait_events_timeout(.1)
        assert data.time==0,'preview must not step physics'
    finally:
        if context is not None:context.free()
        glfw.destroy_window(window);glfw.terminate()

    return dict(physics_time=float(data.time),dimensions=preview.dimensions,mass=preview.mass,wheel_count=preview.wheel_count,reload_count=reload_count)
