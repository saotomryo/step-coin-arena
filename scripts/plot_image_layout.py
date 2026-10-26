import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,FancyArrowPatch
c=json.loads(Path('configs/framework_image_layout.json').read_text());fig,ax=plt.subplots(figsize=(9,8))
for r,color in [(6.875,'gray'),(4.65,'black'),(3.25,'#bc3369'),(1.85,'#60923f')]:ax.add_patch(Circle((0,0),r,fill=False,color=color,lw=2))
for coin in c['competition']['coins']:
    x,y,_=coin['pos'];ax.add_patch(Circle((x,y),.18,color='#edb32b',alpha=.8));ax.text(x+.23,y,str(coin['id']),fontsize=9)
for name,color in [('self','#2678bc'),('opponent','#c44742')]:
    x,y,_=c['simulation']['spawns'][name]['pos'];ax.plot(x,y,'s',color=color,markersize=14);ax.text(x-.1,y+.35,name,color=color,ha='center')
    spec=c['agents'][name]
    for stage,(r,angle) in enumerate(zip([4,2.6,.95],spec['gate_angles'])):
        import math
        start=((r+.65)*math.cos(angle),(r+.65)*math.sin(angle));end=(r*math.cos(angle),r*math.sin(angle))
        ax.add_patch(FancyArrowPatch(start,end,arrowstyle='->',mutation_scale=15,color=color,lw=2))
ax.set(xlim=(-7.4,7.4),ylim=(-7.2,7.2),aspect='equal',xlabel='X [m]',ylabel='Y [m]',title='Photo-derived provisional layout: 9 + 7 + 2 spawn locations')
ax.grid(alpha=.15);fig.tight_layout();fig.savefig('docs/image-layout-plan.png',dpi=150)
