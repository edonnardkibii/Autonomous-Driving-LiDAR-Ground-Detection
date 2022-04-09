import driving_mode
import readchar
import os

if __name__ == '__main__':
    print("Welcome to PiCar")
    key = ""
    while True:
        print("Main Menu:")
        print("Select the driving mode for the PiCar.")
        print("Press '1' for autonomous driving, '2' for manual driving or '3' for mirror mode")
        print("While choosing autonomous/manual driving, ensure the mirrors are removed")
        print("While choosing mirror mode, ensure the mirrors are put in place")
        print("To exit the program, press 'x'")

        key = input("")
        if key != 'x':
            if key == '1':
                # print('Please remove mirrors')
                driving_mode.run_autonomous_mode()
            elif key == '2':
                # print('Please remove mirrors')
                driving_mode.run_manual_mode()
            elif key == '3':
                # print('Please insert mirrors')
                driving_mode.run_ground_detection_mode()
            else:
                print("Invalid input")

            # os.system('clear')
        else:
            break
