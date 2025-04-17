class PIDController:

    def __init__(self):
        self.Kp = None
        self.Ki = None
        self.Kd = None
        self.prev_e = 0.0
        self.prev_int = 0.0
        # #apparently in the code when the PIDcontroll starts there is a callback with error but we cannot pass it to the run function         

    # def run(self, error, v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
    def run(self, theta_ref, theta_hat, delta_t):
        """
        Args:
            v_0 (:double:) linear Duckiebot speed (given).
            theta_ref (:double:) reference heading pose
            theta_hat (:double:) the current estiamted theta.
            prev_e (:double:) tracking error at previous iteration.
            prev_int (:double:) previous integral error term.
            delta_t (:double:) time interval since last call.
        returns:
            v_0 (:double:) linear velocity of the Duckiebot 
            omega (:double:) angular velocity of the Duckiebot
            e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
            e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
        """
        # Tracking error
        error = -(theta_ref-theta_hat)
        P = self.Kp * error

        # Integral of the error can unexpectedy jump due to polynomial detection
        # Error can 
        if abs(error) < 5:
            I = self.Ki*(self.prev_int + error * delta_t)
            self.prev_int = I
        else:
            I = self.prev_int
        max_int_value = 2
        # anti-windup - preventing the integral error from growing too much
        self.prev_int = min(max(self.prev_int,-max_int_value),max_int_value)
        e_int = max(min(I, 1), -1)

        # derivative of the error
        D = self.Kd*((error - self.prev_e) / delta_t)
        self.prev_e = error
        # PID controller for omega
        omega = P + I + D        
        
        return omega, error, e_int
    
    def set_param(self, params)->None :
        self.Kp = params['Kp']
        self.Ki = params['Ki']
        self.Kd = params['Kd']
    