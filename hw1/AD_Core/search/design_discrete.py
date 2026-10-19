from pathlib import Path
HERE=Path(__file__).resolve().parent
import numpy as np
from scipy.optimize import differential_evolution
m,J,V,lf,lr,Cf,Cr=1573.,2873.,30.,1.1,1.58,80000.,80000.
A=np.array([[0,1,0,0],[0,-2*(Cf+Cr)/(m*V),2*(Cf+Cr)/m,2*(-Cf*lf+Cr*lr)/(m*V)],[0,0,0,1],[0,2*(-Cf*lf+Cr*lr)/(J*V),2*(Cf*lf-Cr*lr)/J,-2*(Cf*lf**2+Cr*lr**2)/(J*V)]])
B=np.array([0,2*Cf/m,0,2*Cf*lf/J]);D=np.array([0,2*(-Cf*lf+Cr*lr)/(m*V)-V,0,-2*(Cf*lf**2+Cr*lr**2)/(J*V)])
h=.01;Ad=np.eye(4)+h*A;Bd=h*B
P=[np.linalg.matrix_power(Ad,i) for i in range(5)]
w=np.linalg.inv(np.column_stack([Bd,Ad@Bd,Ad@Ad@Bd,Ad@Ad@Ad@Bd]))[-1]
n=np.arange(300);theta=np.arctan(5/90)
def assess(q,detail=False):
 r1,a1,r2,a2=q
 poles=np.array([r1*np.exp(1j*a1),r1*np.exp(-1j*a1),r2*np.exp(1j*a2),r2*np.exp(-1j*a2)])
 coeff=np.poly(poles).real
 K=w@sum(coeff[4-i]*P[i] for i in range(5))
 M=Ad-np.outer(Bd,K)
 ev,W=np.linalg.eig(M)
 try:c=np.linalg.solve(W,D*theta)
 except:return 1e6
 x=((ev[None,:]**n[:,None]*c)@W.T).real
 u=-x@K
 rate=np.diff(np.r_[0,u])/h
 end=x[15:18]
 scores=np.array([max(abs(x[:,2]))/.01,max(abs(rate))/25,max(abs(x[99:,0]))/.002,max(abs(x[99:,2]))/.0007,max(abs(end[:,0]))/.002,max(abs(end[:,2]))/.0007])
 if detail:return {'K':K.tolist(),'ratios':scores.tolist(),'z_poles':[[z.real,z.imag] for z in ev]}
 return max(scores)+.001*sum(scores)
import json
for seed in range(2):
 r=differential_evolution(assess,[(.05,.99),(.001,2),(.05,.99),(.001,2)],popsize=20,maxiter=400,seed=seed,tol=1e-8)
 ans=assess(r.x,True);print(json.dumps(ans),flush=True)
 open(HERE/f'discrete_design_{seed}.json','w').write(json.dumps(ans,indent=2))
