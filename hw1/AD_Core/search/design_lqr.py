from pathlib import Path
HERE=Path(__file__).resolve().parent
import numpy as np,json
from scipy.linalg import solve_continuous_are
from scipy.optimize import differential_evolution
m,J,V,a,b,C=1573.,2873.,30.,1.1,1.58,80000.
A=np.array([[0,1,0,0],[0,-4*C/(m*V),4*C/m,2*C*(b-a)/(m*V)],[0,0,0,1],[0,2*C*(b-a)/(J*V),2*C*(a-b)/J,-2*C*(a*a+b*b)/(J*V)]])
B=np.array([0,2*C/m,0,2*C*a/J]);D=np.array([0,2*C*(b-a)/(m*V)-V,0,-2*C*(a*a+b*b)/(J*V)])
t=np.unique(np.r_[np.linspace(0,1.5,1501),.01]);h=.01;theta=np.arctan(5/90)
def objective(logq,detail=False):
 Q=np.diag(10**np.array(logq))
 try:
  P=solve_continuous_are(A,B[:,None],Q,np.eye(1));K=B@P;F=A-np.outer(B,K)
  ev,W=np.linalg.eig(F);c=np.linalg.solve(W,D)
 except:return 1e10
 G=(np.expm1(np.minimum(t,h)[:,None]*ev)/ev)*np.exp(np.maximum(t-h,0)[:,None]*ev)
 x=((G*c)@W.T).real*theta/h
 rd=(t<h)*theta/h
 rate=-(x@F.T+rd[:,None]*D)@K
 ratepeak=max(max(abs(rate)),abs(K@(F@x[np.where(t==h)[0][0]]+D*theta/h)))
 ratios=np.array([max(abs(x[:,2]))/.01,ratepeak/25,max(abs(x[t>=1,0]))/.002,max(abs(x[t>=1,2]))/.0007])
 if detail:return {'Q_diagonal':np.diag(Q).tolist(),'K':K.tolist(),'ratios':ratios.tolist(),'poles':[[v.real,v.imag] for v in ev]}
 return max(ratios)+.0001*sum(ratios)
r=differential_evolution(objective,[(-3,2),(-5,1),(-3,3),(-4,2)],seed=642,popsize=12,maxiter=160,tol=1e-7,polish=True)
ans=objective(r.x,True);print(json.dumps(ans),flush=True)
open(HERE/'new_design.json','w').write(json.dumps(ans,indent=2))
