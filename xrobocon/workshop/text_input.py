"""Route macOS keyboard/IME input through pyglet's actual NSTextView responder."""
def focus_prompt(window,caret):
    caret.on_activate();caret.visible=True
    native_window=getattr(window,'_nswindow',None)
    if native_window is None:return dict(backend='pyglet',focused=True)
    view=getattr(window,'_nsview',None)
    text_view=getattr(view,'_textview',None)
    if text_view is None:raise RuntimeError('macOSのテキスト入力ビューを取得できません')
    # PygletView.keyDown only publishes key symbols. NSTextView interprets
    # keyboard layout/IME and publishes on_text / on_text_motion instead.
    if not native_window.makeFirstResponder_(text_view):
        raise RuntimeError('macOSのテキスト入力フォーカスを設定できません')
    actual=native_window.firstResponder()
    actual_pointer=getattr(getattr(actual,'ptr',None),'value',None)
    expected_pointer=getattr(getattr(text_view,'ptr',None),'value',None)
    if actual is not text_view and (actual_pointer is None or actual_pointer!=expected_pointer):
        raise RuntimeError('macOSの入力先がテキストビューに切り替わりませんでした')
    return dict(backend='NSTextView',focused=True)
