from gpiozero import Motor
from time import sleep

class RobotMotor:
    def __init__(self, EnaA, In1A, In2A, EnaB, In1B, In2B):
        self.motorA = Motor(forward=In1A, backward=In2A, pwm=True)
        self.motorB = Motor(forward=In1B, backward=In2B, pwm=True)
        self.mySpeed = 0

    def move(self, speed=0.5, turn=0, t=0):
        speed = max(min(speed, 1), -1)  # Clamp speed to [-1, 1]
        turn = max(min(turn, 1), -1)    # Clamp turn to [-1, 1]
        leftSpeed = speed - turn
        rightSpeed = speed + turn

        self.motorA.forward(leftSpeed) if leftSpeed > 0 else self.motorA.backward(-leftSpeed)
        self.motorB.forward(rightSpeed) if rightSpeed > 0 else self.motorB.backward(-rightSpeed)
        sleep(t)

    def stop(self, t=0):
        self.motorA.stop()
        self.motorB.stop()
        self.mySpeed = 0
        sleep(t)

def main():
    motor.move(0.5, 0, 2)
    motor.stop(2)
    motor.move(-0.5, 0, 2)
    motor.stop(2)
    motor.move(0, 0.5, 2)
    motor.stop(2)
    motor.move(0, -0.5, 2)
    motor.stop(2)

if __name__ == '__main__':
    motor = RobotMotor(2, 3, 4, 17, 22, 27)
    main()
