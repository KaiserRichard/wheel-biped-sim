from types import SimpleNamespace
from .sensors import Sensors
class AssemblySensors(Sensors):
    def __init__(self,sim,seed):
        self.sim=sim
        super().__init__(sim,seed)
    def _packet(self,data):
        q,v=self.sim.encoder_arrays(data)
        return super()._packet(SimpleNamespace(qpos=q,qvel=v,sensordata=data.sensordata))
