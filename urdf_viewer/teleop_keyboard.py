#!/usr/bin/env python3

import sys
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    """
    キーボードからの入力に基づいてTwistメッセージを発行するノード。
    ロボットをキーボードで操作するために使用します。
    """

    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # パブリッシャーの設定
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # パラメータ
        self.linear_speed = 0.2  # 直線速度 (m/s)
        self.angular_speed = 0.5  # 角速度 (rad/s)
        
        self.get_logger().info('キーボード操作ノードが起動しました')
        self.get_logger().info('操作方法:')
        self.get_logger().info('  w: 前進')
        self.get_logger().info('  s: 後退')
        self.get_logger().info('  a: 左回転')
        self.get_logger().info('  d: 右回転')
        self.get_logger().info('  スペース: 停止')
        self.get_logger().info('  q: 終了')

    def get_key(self):
        """
        ユーザーからのキー入力を1文字取得
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """
        メインループ。キー入力を監視し、対応するコマンドを発行します
        """
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # qが押されたらプログラムを終了
                if key == 'q':
                    break
                
                twist = Twist()
                
                # キー入力に応じて速度を設定
                if key == 'w':
                    twist.linear.x = self.linear_speed  # 前進
                elif key == 's':
                    twist.linear.x = -self.linear_speed  # 後退
                elif key == 'a':
                    twist.angular.z = self.angular_speed  # 左回転
                elif key == 'd':
                    twist.angular.z = -self.angular_speed  # 右回転
                elif key == ' ':  # スペースで停止
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                # 速度コマンドを発行
                self.publisher.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')
        
        finally:
            # 停止コマンドを発行してから終了
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            # 元のターミナル設定に戻す
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    
    # クリーンアップ
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()