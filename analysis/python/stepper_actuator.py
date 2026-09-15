from dataclasses import dataclass

import numpy as np

from cartpole_model import (
    CartPoleParams,
    nonlinear_dynamics,
)


@dataclass(frozen=True)
class StepperActuatorParams:

    # 36 teeth * 2 mm/tooth = 72 mm per motor revolution
    pulley_travel_per_rev_m: float = 0.072

    # practical limits
    max_motor_velocity_rad_s: float = 100.0
    max_motor_acceleration_rad_s2: float = 100.0
    steps_per_rev: int = 1600

    @property
    def meters_per_motor_rad(self) -> float:
        return (self.pulley_travel_per_rev_m / (2.0 * np.pi))

    @property
    def max_cart_velocity_m_s(self) -> float:
        return (self.meters_per_motor_rad * self.max_motor_velocity_rad_s)

    @property
    def max_cart_acceleration_m_s2(self) -> float:
        return (self.meters_per_motor_rad *self.max_motor_acceleration_rad_s2)


@dataclass(frozen=True)
class StepperCommand:

    requested_force_n: float
    ideal_cart_acceleration_m_s2: float
    requested_motor_acceleration_rad_s2: float
    applied_motor_acceleration_rad_s2: float
    target_motor_velocity_rad_s: float
    target_cart_velocity_m_s: float
    target_step_frequency_hz: float


def force_to_ideal_cart_acceleration(
    state: np.ndarray,
    requested_force_n: float,
    plant_params: CartPoleParams,
) -> float:

    """
    Ask for force F, get ideal cart acceleration
    """

    state_derivative = nonlinear_dynamics(
        t=0.0,
        state=state,
        params=plant_params,
        input_force_n=requested_force_n,
    )

    return float(
        state_derivative[1]
    )


def cart_acceleration_to_motor_acceleration(
    cart_acceleration_m_s2: float,
    actuator_params: StepperActuatorParams,
) -> float:

    """
    x = k_x * theta_m
    therefore
        x_ddot = k_x * alpha_m
    so
        alpha_m = x_ddot / k_x
    """

    return (cart_acceleration_m_s2 / actuator_params.meters_per_motor_rad)


def motor_velocity_to_cart_velocity(
    motor_velocity_rad_s: float,
    actuator_params: StepperActuatorParams,
) -> float:

    return (actuator_params.meters_per_motor_rad * motor_velocity_rad_s)


def motor_velocity_to_step_frequency(
    motor_velocity_rad_s: float,
    actuator_params: StepperActuatorParams,
) -> float:

    revolutions_per_second = (abs(motor_velocity_rad_s) /(2.0 * np.pi))

    return (revolutions_per_second *actuator_params.steps_per_rev)


def build_stepper_command(
    state: np.ndarray,
    requested_force_n: float,
    current_target_motor_velocity_rad_s: float,
    dt_seconds: float,
    plant_params: CartPoleParams,
    actuator_params: StepperActuatorParams,
) -> StepperCommand:

    # Force from LQR -> ideal cart acceleration

    ideal_cart_acceleration = (
        force_to_ideal_cart_acceleration(
            state=state,
            requested_force_n=requested_force_n,
            plant_params=plant_params,
        )
    )


    # 2. Desired cart acceleration -> desired motor angular acceleration

    requested_motor_acceleration = (
        cart_acceleration_to_motor_acceleration(
            ideal_cart_acceleration,
            actuator_params,
        )
    )


    # Respect acceleration capability

    applied_motor_acceleration = float(
        np.clip(
            requested_motor_acceleration,
            -actuator_params.max_motor_acceleration_rad_s2,
            actuator_params.max_motor_acceleration_rad_s2,
        )
    )


    # integrate acceleration 
    # omega_new = omega_old + alpha * dt

    unclipped_motor_velocity = (
        current_target_motor_velocity_rad_s
        +
        applied_motor_acceleration
        * dt_seconds
    )

    # Respect velocity capability

    new_target_motor_velocity = float(
        np.clip(
            unclipped_motor_velocity,

            -actuator_params.max_motor_velocity_rad_s,

            actuator_params.max_motor_velocity_rad_s,
        )
    )

    #recalculate actually achievable acceleration after velocity saturation
    
    applied_motor_acceleration = (
        new_target_motor_velocity
        -
        current_target_motor_velocity_rad_s
    ) / dt_seconds


    # Convert velocity to useful physicals

    target_cart_velocity = (
        motor_velocity_to_cart_velocity(
            new_target_motor_velocity,
            actuator_params,
        )
    )

    target_step_frequency = (
        motor_velocity_to_step_frequency(
            new_target_motor_velocity,
            actuator_params,
        )
    )

    return StepperCommand(
        requested_force_n=requested_force_n,

        ideal_cart_acceleration_m_s2=
            ideal_cart_acceleration,

        requested_motor_acceleration_rad_s2=
            requested_motor_acceleration,

        applied_motor_acceleration_rad_s2=
            applied_motor_acceleration,

        target_motor_velocity_rad_s=
            new_target_motor_velocity,

        target_cart_velocity_m_s=
            target_cart_velocity,

        target_step_frequency_hz=
            target_step_frequency,
    )