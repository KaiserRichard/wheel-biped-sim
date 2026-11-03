"""MEASURED projected assembly matrices; see numerical provenance in docs."""
import json
from pathlib import Path
import numpy as np
from .model import constants
TABLES=json.loads((Path(__file__).resolve().parents[2]/"models/assembly_gains.json").read_text())["variants"]
def linearize(p,L,phi0=0.):
    rows=TABLES["built"]["samples"]
    lengths=np.array([r["L_m"] for r in rows])
    j=int(np.clip(np.searchsorted(lengths,L)-1,0,len(lengths)-2))
    w=float(np.clip((L-lengths[j])/(lengths[j+1]-lengths[j]),0,1))
    return tuple((1-w)*np.array(rows[j][k])+w*np.array(rows[j+1][k]) for k in ("A","B","x0","u0"))
def equilibrium(p,L,phi0=0.):
    _,_,x,u=linearize(p,L)
    return x[2],0.,u[1]
def coefficients():
    return np.array(TABLES["built"]["polyfit_coefficients"])
