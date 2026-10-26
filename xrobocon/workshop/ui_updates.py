"""Small invalidation helpers: idle editor windows need no continuous GL draws."""
class DrawGate:
    def __init__(self):self.dirty=True;self.draw_count=0
    def invalidate(self):self.dirty=True
    def draw(self,callback,dt):
        if not self.dirty:return False
        self.dirty=False;callback(dt);self.draw_count+=1
        return True

class ProposalCache:
    def __init__(self):self.references=None;self.text='';self.format_count=0
    def render(self,controller,formatter):
        references=(controller.current,controller.proposal,controller.last_result)
        if self.references is None or any(a is not b for a,b in zip(references,self.references)):
            self.references=references;self.text=formatter(controller);self.format_count+=1
        return self.text

def set_if_changed(target,attribute,value,invalidate):
    if getattr(target,attribute)!=value:
        setattr(target,attribute,value);invalidate();return True
    return False
