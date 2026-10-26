"""Optional competition overlay. The physics adapter remains competition-free."""
from xrobocon.native_panel import decision_status
import queue
from xrobocon.native_panel import ACTIONS,LABELS,allowed

class ViewerPanel:
    def __init__(self,viewer,screenshot=None):
        self.viewer=viewer;self.commands=queue.Queue();self.state={};self.screenshot=screenshot
        self.plugin=None;self.error=None;self.drawn=False
    def start(self):
        from genesis.vis.viewer_plugins import ViewerPlugin
        owner=self
        class Plugin(ViewerPlugin):
            def __init__(self):
                super().__init__();self.batch=None;self.texts=[];self.buttons=[]
            def create(self):
                import pyglet
                from pyglet import shapes
                self.batch=pyglet.graphics.Batch()
                self.background=shapes.Rectangle(0,0,360,440,color=(20,27,39),batch=self.batch,group=pyglet.graphics.Group(order=0))
                self.texts=[pyglet.text.Label('',font_name='Hiragino Sans',font_size=13,x=0,y=0,batch=self.batch,group=pyglet.graphics.Group(order=2)) for _ in range(12)]
                for action,text in ACTIONS:
                    rect=shapes.Rectangle(0,0,160,35,color=(50,85,125),batch=self.batch,group=pyglet.graphics.Group(order=1))
                    caption=pyglet.text.Label(text,font_name='Hiragino Sans',font_size=12,x=0,y=0,batch=self.batch,group=pyglet.graphics.Group(order=2))
                    self.buttons.append((action,rect,caption))
            def on_draw(self):
                import pyglet
                from pyglet import gl
                from pyglet.math import Mat4
                try:
                    if self.batch is None:self.create()
                    w,h=self.viewer.get_size();x=max(0,w-380);y=max(0,h-460)
                    self.background.position=(x,y)
                    s=owner.state;status=s.get('control_status','ready');coins=s.get('coins',[])
                    values=['ステップコイン・アリーナ 競技（青 / 赤）',f"{LABELS.get(status,status)} 第{s.get('round_id',0)}試合",f"残り {s.get('remaining',0):.1f} 秒",f"青 {s.get('scores',{}).get('self',0)} 点 / 赤 {s.get('scores',{}).get('opponent',0)} 点",f"出現中 {sum(c['status']=='available' for c in coins)} / 取得済み {sum(c['status']=='collected' for c in coins)}"]
                    for i,t in enumerate(self.texts[:5]):t.text=values[i];t.x=x+12;t.y=y+415-i*27
                    for i,(action,rect,caption) in enumerate(self.buttons):
                        rect.position=(x+12+i%2*175,y+220-i//2*45)
                        enabled=allowed(action,status);rect.color=(50,85,125) if enabled else (35,40,50)
                        caption.x=rect.x+8;caption.y=rect.y+11;caption.color=(240,245,255,255) if enabled else (110,115,125,255)
                    events=list(reversed(s.get('events',[])))[:5]
                    for i,t in enumerate(self.texts[5:10]):
                        e=events[i] if i<len(events) else None
                        t.text=(f"{e['time']:.1f}s {dict(self='青',opponent='赤').get(e.get('participant'),'')} スタート復帰" if e and e['type']=='robot_recovered' else f"{e['time']:.1f}s {e['type']} {e.get('coin_id','')}" if e else '')
                        t.x=x+12;t.y=y+108-i*19
                    self.texts[10].text=str(s.get('error') or decision_status(s) or f"スタート復帰 青:{s.get('recovery_counts',{}).get('self',0)} 赤:{s.get('recovery_counts',{}).get('opponent',0)}")[:35];self.texts[10].x=x+12;self.texts[10].y=y+10
                    projection=self.viewer.projection;view=self.viewer.view
                    depth=gl.glIsEnabled(gl.GL_DEPTH_TEST)
                    try:
                        self.viewer.projection=Mat4.orthogonal_projection(0,w,0,h,-1,1);self.viewer.view=Mat4()
                        gl.glDisable(gl.GL_DEPTH_TEST);self.batch.draw()
                        if owner.screenshot and not owner.drawn and s:
                            pyglet.image.get_buffer_manager().get_color_buffer().save(owner.screenshot)
                        owner.drawn=True
                    finally:
                        self.viewer.projection=projection;self.viewer.view=view
                        if depth:gl.glEnable(gl.GL_DEPTH_TEST)
                except Exception as error:
                    owner.error=str(error)
            def contains(self,x,y):
                return self.background.x<=x<=self.background.x+360 and self.background.y<=y<=self.background.y+440 if self.batch else False
            def on_mouse_press(self,x,y,button,modifiers):
                if not self.contains(x,y):return
                if button==1:
                    for action,rect,_ in self.buttons:
                        if rect.x<=x<=rect.x+160 and rect.y<=y<=rect.y+35 and allowed(action,owner.state.get('control_status','ready')):
                            owner.commands.put(action);break
                return True
            def on_mouse_drag(self,x,y,dx,dy,buttons,modifiers):
                if self.contains(x,y):return True
            def on_mouse_scroll(self,x,y,dx,dy):
                if self.contains(x,y):return True
            def on_close(self):owner.commands.put('shutdown')
        self.plugin=Plugin();self.viewer.add_plugin(self.plugin)
    def close(self):
        if self.plugin and self.viewer.is_alive():self.viewer.remove_plugin(self.plugin)
