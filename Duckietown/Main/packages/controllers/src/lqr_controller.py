import numpy as np

class LQRController:

    def __init__(self):
        self.K = None  # LQR gain
        self.A = None
        self.B = None

    def run(self, theta_ref, theta_hat, delta_t):
        """
        Args:
            theta_ref (float): reference heading
            theta_hat (float): current heading estimate
            delta_t (float): time step

        Returns:
            omega (float): angular velocity
            error (float): current error (for logging/feedback)
            0.0: dummy value (to keep return structure)
        """
        # error state
        e = -(theta_ref - theta_hat)
        x = np.array([[e]])

        # omega = -Kx
        omega = float(-self.K @ x)

        return omega, e, 0.0

    def set_param(self, params):
        """
        Set system matrices A, B and LQR weights Q, R
        """
        A = np.array([[1]])  # simplified: next error = current error (identity)
        B = np.array([[1]])  # control directly affects the error

        Q = np.array([[params.get('Q', 1.0)]])
        R = np.array([[params.get('R', 0.1)]])

        # Solve Riccati equation manually (for 1D system)
        # P = solution to Riccati equation for LQR
        # K = (B^T P B + R)^-1 B^T P A
        # For scalar A, B: K = (R + Bᵗ P B)⁻¹ Bᵗ P A

        # For 1D, we can analytically solve the discrete algebraic Riccati equation:
        P = Q
        K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A

        self.A = A
        self.B = B
        self.K = K

