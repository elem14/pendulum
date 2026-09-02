from dataclasses import dataclass

@dataclass
class Params:
    m_c: float     # carriage mass [kg]
    m_p: float     # pendulum rod mass [kg]
    L: float       # full rod length [m]
    g: float       # gravitational acceleration [m/s^2]
    b_x: float     # viscous friction, carriage/rail [N.s/m]
    b_theta: float # viscous friction, pivot bearing [N.m.s/rad]

    @property
    def l_c(self) -> float:
        return self.L / 2.0

    @property
    def I_cm(self) -> float:
        return (1.0 / 12.0) * self.m_p * self.L ** 2

    @property
    def I_p(self) -> float:
        return self.I_cm + self.m_p * self.l_c ** 2  # == (1/3) m_p L^2


_steel_density = 7850.0
_rod_od = 0.25 * 0.0254
_rod_radius = _rod_od / 2.0
_rod_length = 12.0 * 0.0254
_rod_volume = 3.141592653589793 * _rod_radius ** 2 * _rod_length
_rod_mass_guess = _steel_density * _rod_volume

DEFAULT_PARAMS = Params(
    m_c=0.30, 
    m_p=_rod_mass_guess, 
    L = _rod_length, 
    g=9.81, 
    b_x=0.10, 
    b_theta=0.001,
)