"""Native Python editor: prompt -> review -> validate -> preview/test -> save."""
import json,time
from pathlib import Path

def proposal_text(controller):
    if controller.proposal is not None:
        p=controller.proposal;lines=[p['title'],'',p['summary']]
        result=controller.last_result
        if result:
            lines.extend(['','現在モデルの診断結果:'])
            for candidate in result.get('results',[]):
                lines.append('合格' if candidate.get('passed') else '失敗／未完了')
                for trial in candidate.get('trials',[]):
                    lines.append(f"・{trial['case']}: {'合格' if trial['passed'] else '失敗'} / {trial['simulation_seconds']:.2f}秒")
            if result.get('error'):lines.append(str(result['error']))
        lines.extend(['','機体の変更:'])
        names={'pos':'位置 [m]','size':'寸法 [m]','mass':'質量 [kg]','friction':'摩擦','rgba':'色','fromto':'端点 [m]','range':'可動域 [度]','damping':'減衰','armature':'付加慣性'}
        lines.extend(f"・{c['name']} / {names[c['attribute']]} → {c['value']}" for c in p['model_changes'])
        if not p['model_changes']:lines.append('・形状の変更なし')
        lines.extend(['','制御設定の変更:'])
        lines.extend(f"・駆動 {c['key']} → {c['value']}" for c in p['drive_changes'])
        lines.extend(f"・{c['profile']} / {c['key']} → {c['value']}" for c in p['motion_changes'])
        return '\n'.join(lines)
    if controller.current:
        return '現在のモデル・制御設定\n'+json.dumps(dict(source=controller.current['source_robot'],parent=controller.current.get('revision_id'),tuning=controller.current['tuning']),ensure_ascii=False,indent=2)
    return '変更案がここに表示されます。'

def show_workshop(controller,screenshot=None,smoke=False,metrics_path=None):
    import pyglet
    pyglet.options['dpi_scaling']='stretch'
    # Initialize the Cocoa app before creating the first GL window.
    import pyglet.app
    from pyglet import shapes
    window=pyglet.window.Window(1200,830,caption='ステップコイン・アリーナ — ロボット・制御作成',resizable=False)
    from xrobocon.workshop.ui_updates import DrawGate,ProposalCache,set_if_changed
    from xrobocon.workshop.text_input import focus_prompt
    draw_gate=DrawGate();proposal_cache=ProposalCache()
    native_draw=window.draw
    window.draw=lambda dt:draw_gate.draw(native_draw,dt)
    batch=pyglet.graphics.Batch();objects=[];buttons=[]
    def label(text,x,y,size=14,color=(230,235,245,255)):
        v=pyglet.text.Label(text,font_name='Hiragino Sans',font_size=size,x=x,y=y,color=color,batch=batch);objects.append(v);return v
    label('ロボット・制御ワークショップ',24,790,24)
    label('既存機体を基に作成。変更案を確認して適用し、3D確認・診断後に版を保存できます。',24,758,13)
    def button(text,x,y,width,action):
        rect=shapes.Rectangle(x,y,width,38,color=(40,85,130),batch=batch);objects.append(rect)
        caption=label(text,x+10,y+12,12);buttons.append(dict(rect=rect,caption=caption,action=action))
        return caption
    robot_ids=list(controller.catalog.options('robots'));selected=[0];revision_index=[-1]
    controller.choose_robot(robot_ids[0])
    def choose():
        selected[0]=(selected[0]+1)%len(robot_ids);controller.choose_robot(robot_ids[selected[0]])
    def load_saved():
        versions=controller.library.list()
        if not versions:raise ValueError('保存版がありません。変更案を適用して保存してください。')
        revision_index[0]=(revision_index[0]+1)%len(versions);controller.load_revision(versions[revision_index[0]].parent)
    source_label=button('元機体',24,702,450,choose)
    saved_label=button('保存版を読み込む（クリックで切替）',490,702,460,load_saved)
    button('3Dプレビュー',966,702,210,lambda:controller.preview(smoke=smoke))
    label('プロンプト（日本語入力・貼り付け可）',24,673,14)
    boxes=[shapes.Rectangle(24,490,1152,168,color=(28,35,48),batch=batch),shapes.Rectangle(24,198,1152,210,color=(25,30,42),batch=batch)];objects.extend(boxes)
    document=pyglet.text.document.UnformattedDocument('')
    document.set_style(0,0,dict(font_name='Hiragino Sans',font_size=13,color=(240,244,250,255),wrap='char'))
    editor=pyglet.text.layout.IncrementalTextLayout(document,x=34,y=500,width=1132,height=148,multiline=True,batch=batch)
    class EditorCaret(pyglet.text.caret.Caret):
        def _blink(self,dt):
            super()._blink(dt);draw_gate.invalidate()
    caret=EditorCaret(editor,color=(140,205,255),window=window);caret.visible=True
    output_doc=pyglet.text.document.UnformattedDocument('')
    output=pyglet.text.layout.IncrementalTextLayout(output_doc,x=34,y=208,width=1132,height=190,multiline=True,batch=batch)
    def generate(mode):controller.propose(document.text,mode)
    button('機体のみ生成・編集',24,435,220,lambda:generate('robot_only'))
    button('機体＋制御の変更案を生成',260,435,270,lambda:generate('robot'))
    button('制御設定だけの変更案を生成',546,435,270,lambda:generate('control'))
    button('生成・診断を取り消す',832,435,220,controller.cancel)
    label('GPT-6 Luna / low',985,673,11)
    button('1. 適用・検証',24,142,180,controller.apply)
    button('2. 別の版として保存',220,142,250,controller.save)
    button('3. 対戦へ登録',486,142,200,controller.register)
    button('平地診断（直進・旋回・経路）',24,86,300,lambda:controller.diagnostic('flat'))
    button('三段登坂の診断',340,86,220,lambda:controller.diagnostic('steps'))
    status=pyglet.text.Label('',font_name='Hiragino Sans',font_size=11,x=24,y=58,width=1150,multiline=True,batch=batch,color=(255,200,135,255))
    label('新しい関節・機構・制御コードの自動生成は今後拡張。診断とプレビューは別ウィンドウ／プロセスです。',24,15,11,(165,180,200,255))
    old_output=[None];smoke_error=[];started=time.monotonic();smoke_phase=[0]
    metrics=dict(text_event_ms=[],preview_typing=False,inference_typing=False)
    def invoke(index):
        b=buttons[index];r=b['rect'];on_mouse_press(r.x+8,r.y+8,1,0)
    def refresh(dt):
        controller.poll()
        set_if_changed(source_label,'text','元機体: '+controller.current['entry']['label'][:23],draw_gate.invalidate)
        set_if_changed(saved_label,'text','保存版: '+(controller.directory.name[:24] if controller.saved else '未保存／クリックで過去の版を読込'),draw_gate.invalidate)
        set_if_changed(status,'text',controller.message,draw_gate.invalidate)
        text=proposal_cache.render(controller,proposal_text)
        if text!=old_output[0]:
            output_doc.text=text;output_doc.set_style(0,len(text),dict(font_name='Hiragino Sans',font_size=11,color=(225,235,245,255),wrap='char'))
            output.view_y=0;old_output[0]=text;draw_gate.invalidate()
        for b in buttons:
            set_if_changed(b['rect'],'color',(45,50,60,255) if controller.busy and b['action']!=controller.cancel else (40,85,130,255),draw_gate.invalidate)
        if smoke:
            try:
                if time.monotonic()-started>30:raise TimeoutError('workshop UI smoke timed out')
                if smoke_phase[0]==0:
                    if time.monotonic()-started<1.5:return
                    metrics['idle_seconds']=round(time.monotonic()-started,3);metrics['idle_draw_count']=draw_gate.draw_count
                    assert controller.directory is None
                    invoke(2);assert controller.process is not None,controller.message
                    on_text('プレビュー中も入力できます。');assert document.text.endswith('入力できます。')
                    metrics['preview_typing']=True
                    smoke_phase[0]=1
                elif smoke_phase[0]==1 and not controller.busy:
                    assert controller.directory is None
                    document.text='';caret.position=0
                    for _ in range(100):on_text('入力テスト。')
                    document.text='';caret.position=0
                    on_text('車体を1cm下げてください。制御設定は維持してください。')
                    on_text_motion(pyglet.window.key.MOTION_LEFT);on_text_motion(pyglet.window.key.MOTION_RIGHT)
                    assert document.text.endswith('ください。')
                    invoke(3)
                    submitted=controller.prompt;on_text('次の指示を入力中。')
                    assert controller.busy and controller.prompt==submitted and document.text.endswith('入力中。')
                    metrics['inference_typing']=True;smoke_phase[0]=2
                elif smoke_phase[0]==2 and not controller.busy:
                    assert controller.proposal is not None,controller.message
                    invoke(7);assert controller.directory is not None,controller.message
                    invoke(8);assert controller.saved,controller.message
                    invoke(9);assert (controller.library.root/'catalog.json').exists(),controller.message
                    refresh_output=proposal_text(controller);output_doc.text=refresh_output
                    output_doc.set_style(0,len(refresh_output),dict(font_name='Hiragino Sans',font_size=11,color=(225,235,245,255),wrap='char'))
                    smoke_phase[0]=3
                elif smoke_phase[0]==3:
                    window.switch_to();window.clear();batch.draw()
                    if screenshot:pyglet.image.get_buffer_manager().get_color_buffer().save(str(screenshot))
                    window.close()
            except Exception as error:smoke_error.append(error);window.close()
    def activate_prompt():
        try:
            metrics['native_input_focus']=focus_prompt(window,caret)
        except RuntimeError as error:controller.message=str(error)
        draw_gate.invalidate()
    pyglet.clock.schedule_once(lambda dt:activate_prompt(),0)
    @window.event
    def on_expose():draw_gate.invalidate()
    @window.event
    def on_activate():activate_prompt()
    @window.event
    def on_deactivate():caret.on_deactivate();draw_gate.invalidate()
    @window.event
    def on_draw():window.clear();batch.draw()
    @window.event
    def on_text(text):
        began=time.perf_counter();caret.on_text(text);draw_gate.invalidate()
        if smoke:metrics['text_event_ms'].append((time.perf_counter()-began)*1000)
    @window.event
    def on_text_motion(motion):
        caret.on_text_motion(motion);draw_gate.invalidate()
    @window.event
    def on_text_motion_select(motion):
        caret.on_text_motion_select(motion);draw_gate.invalidate()
    @window.event
    def on_mouse_press(x,y,button,modifiers):
        draw_gate.invalidate()
        if 24<=x<=1176 and 490<=y<=658:
            activate_prompt();caret.on_mouse_press(x,y,button,modifiers);return
        for b in buttons:
            r=b['rect']
            if r.x<=x<=r.x+r.width and r.y<=y<=r.y+r.height:
                try:b['action']()
                except Exception as error:controller.message=str(error)
                return
    @window.event
    def on_mouse_drag(x,y,dx,dy,buttons,modifiers):
        draw_gate.invalidate()
        if 490<=y<=658:caret.on_mouse_drag(x,y,dx,dy,buttons,modifiers)
    @window.event
    def on_mouse_scroll(x,y,sx,sy):
        draw_gate.invalidate()
        if 198<=y<=408:output.view_y=max(min(0,output.height-output.content_height),min(0,output.view_y+sy*30))
        elif 490<=y<=658:caret.on_mouse_scroll(x,y,sx,sy)
    pyglet.gl.glClearColor(.06,.08,.12,1);pyglet.clock.schedule_interval(refresh,.1)
    try:pyglet.app.run()
    finally:controller.close()
    metrics.update(draw_count=draw_gate.draw_count,proposal_format_count=proposal_cache.format_count)
    if metrics_path:Path(metrics_path).write_text(json.dumps(metrics,indent=2))
    if smoke_error:raise smoke_error[0]
    return metrics
