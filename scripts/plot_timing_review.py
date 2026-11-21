#!/usr/bin/env python3
"""Generate explanatory figures (requires matplotlib); calculations, not measurements."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

out=Path(__file__).resolve().parents[1]/'docs/assets'
out.mkdir(exist_ok=True)
plt.rcParams.update({'figure.facecolor':'#101820','axes.facecolor':'#101820','text.color':'#edf3f8','axes.labelcolor':'#bac8d4','xtick.color':'#bac8d4','ytick.color':'#bac8d4','axes.edgecolor':'#415363','font.size':11,'savefig.facecolor':'#101820'})
fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
r=np.linspace(0,100,301)
ax.plot(r,1000/(33+r),label='v0.0.3: request time + 33 ms',color='#ff8b7b',lw=2.5)
ax.plot(r,1000/np.maximum(33,r),label='Updated: 33 ms total target, sequential requests',color='#d7ff45',lw=2.5)
ax.scatter([30],[1000/63],color='#ff8b7b',s=50)
ax.annotate('Illustrative 30 ms request\n15.9 Hz → 30.3 Hz ceiling',xy=(30,1000/63),xytext=(43,23),arrowprops={'arrowstyle':'->','color':'#bac8d4'},fontsize=11)
ax.set(xlabel='Request + processing duration (ms)',ylabel='Polling rate (Hz)',ylim=(0,34),xlim=(0,100),title='Why a 30 Hz target can look like 16 Hz')
ax.grid(alpha=.15);ax.legend(loc='lower left',frameon=False,labelcolor='#edf3f8')
fig.text(.5,-.035,'Calculated scheduling limits, not measured Wi-Fi performance. Fresh packet rate can be lower.',ha='center',color='#bac8d4',fontsize=10)
fig.savefig(out/'dashboard-rate-budget.png',dpi=160,bbox_inches='tight');plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
ax=axes[0];t=np.array([0,40]);dist=np.array([-.6,.2])
ax.axhline(0,color='#d7ff45',lw=1.5);ax.plot(t,dist,'o-',color='#edf3f8',lw=2);ax.scatter([30],[0],s=70,color='#d7ff45',zorder=5)
ax.annotate('Interpolated crossing\n30 ms after first fix',xy=(30,0),xytext=(4,.24),arrowprops={'arrowstyle':'->','color':'#bac8d4'},fontsize=10)
ax.set(xlabel='Time since first fix (ms)',ylabel='Signed distance to timing line (m)',title='Crossing between GPS fixes',xlim=(-3,43),ylim=(-.7,.5));ax.grid(alpha=.15)
ax=axes[1];mph=np.linspace(20,100,100)
for error,color in [(.25,'#d7ff45'),(.5,'#61c6db'),(1,'#ff8b7b')]:ax.plot(mph,error/(mph*.44704)*1000,label=f'{error:g} m line-normal error',color=color,lw=2)
ax.set(xlabel='Speed perpendicular to the timing line (mph)',ylabel='Crossing-time sensitivity (ms)',title='Position error still matters',xlim=(20,100),ylim=(0,120));ax.grid(alpha=.15);ax.legend(frameon=False,labelcolor='#edf3f8',fontsize=9)
fig.text(.5,-.04,'Illustrative geometry and error sensitivity only. Curves are not Blackbox accuracy estimates.',ha='center',color='#bac8d4',fontsize=11)
fig.savefig(out/'crossing-maths.png',dpi=160,bbox_inches='tight')
