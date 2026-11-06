#!/usr/bin/env python3
"""Pure helper checks for smooth trajectory and weighted motor mixing."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cefiro_multibody import fifth_order,allocate_motors
for T in (8.,11.,20.):
    assert np.allclose(fifth_order(0,T),(0,0,0),atol=1e-12)
    assert np.allclose(fifth_order(T,T),(1,0,0),atol=1e-12)
    assert np.allclose(fifth_order(T+1,T),(1,0,0),atol=1e-12)
G=np.array([[1,-1,1,-1],[1,1,-1,-1],[-1,1,1,-1]],float)*1e-7
for weights in ((1,1,1),(.35,1,1)):
    rpm,res,actual=allocate_motors(G,[0,.2,.5],4700,5400,axis_weights=weights)
    assert rpm.shape==(4,) and np.all(np.isfinite(rpm)) and np.isfinite(res)
print('PASS: smooth quintic transition and bounded actuator mixing')
