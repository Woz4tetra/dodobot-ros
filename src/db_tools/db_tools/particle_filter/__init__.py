import warnings
try:
    from .jit_particle_filter import JitParticleFilter
except ImportError as e:
    warnings.warn("Cannot import '%s'" % e)
from .particle_filter import ParticleFilter, FilterSerial
