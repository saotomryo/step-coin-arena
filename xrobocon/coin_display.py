"""Presentation of authoritative coin state; never awards or schedules coins."""
class CoinDisplay:
    def __init__(self,positions,set_position):
        self.positions={key:tuple(pos) for key,pos in positions.items()}
        self.set_position=set_position;self.visible={key:None for key in positions}
    def invalidate(self):
        """Force projection after an external physics reset changes entity transforms."""
        self.visible={key:None for key in self.positions}
    def sync(self,coins):
        by_id={c['id']:c for c in coins}
        if set(by_id)!=set(self.positions):raise ValueError('display and competition coin ids must match')
        changes=[]
        for key,pos in self.positions.items():
            visible=by_id[key]['status']=='available'
            if self.visible[key]==visible:continue
            self.set_position(key,pos if visible else (pos[0],pos[1],-20.0))
            self.visible[key]=visible;changes.append(dict(coin_id=key,visible=visible))
        return changes
