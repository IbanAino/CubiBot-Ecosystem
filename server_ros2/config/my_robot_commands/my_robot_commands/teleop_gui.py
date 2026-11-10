import rclpy

from my_robot_commands.model import CommandModel
from my_robot_commands.view import TeleopView
from my_robot_commands.controller import TeleopController


def main(args=None):
    rclpy.init(args=args)

    model = CommandModel()
    controller = TeleopController(model)
    view = TeleopView(on_send_text=controller.handle_send_text)

    def on_close():
        model.destroy_node()
        rclpy.shutdown()
        view.destroy()

    view.protocol('WM_DELETE_WINDOW', on_close)
    view.mainloop()


if __name__ == '__main__':
    main()
