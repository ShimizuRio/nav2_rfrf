#! /bin/python3
import rclpy
from rclpy.node import Node
import pexpect
import sys

# from bluepy.btle import Scanner, DefaultDelegate
import binascii
import copy
from std_msgs.msg import String, Int8, Bool
import time


def trigger_device(device):

    # [mac, dev_type, act] = (
    #     device  # デバイス情報（MACアドレス、デバイスタイプ、アクション）を取得
    # )
    # # print 'Start to control'  # デバッグ用の出力

    # # con = pexpect.spawn('gatttool -b ' + mac + ' -t random -I')  # gatttoolを使用してデバイスに接続するためのプロセスを開始
    # # con.expect('\[LE\]>')  # gatttoolの準備完了を待つ
    # # print('Preparing to connect.')

    # retry = 2  # 接続試行回数を設定
    # index = 0  # 接続状態を示すインデックス
    # print("Preparing to connect..." + mac)
    # while retry > 0 and 0 == index:  # 接続が成功するか、試行回数が尽きるまで繰り返す
    #     con = pexpect.spawn(
    #         "gatttool -b " + mac + " -t random -I"
    #     )  # gatttoolを使用してデバイスに接続するためのプロセスを開始
    #     con.expect("\[LE\]>")  # gatttoolの準備完了を待つ
    #     print("Preparing to connect...")
    #     con.sendline("connect")  # デバイスに接続を試みる
    #     # 異なるBluezバージョンに対応するために複数の接続成功メッセージを待つ
    #     try:
    #         index = con.expect(
    #             ["Error", "\[CON\]", "Connection successful.*\[LE\]>"], 15
    #         )
    #         if 2 == index:
    #             break

    #     except pexpect.exceptions.TIMEOUT:
    #         print("Connection timeout")
    #         return False

    #     retry -= 1  # 接続試行回数を減らす
    #     # if retry == 0:
    #     #     return False  # （コメントアウト）接続試行回数が尽きたら失敗を返す

    # if 0 == index:  # 接続が失敗した場合
    #     print("Connection error.")  # エラーメッセージを表示
    #     return False  # プログラムを終了

    # print("Connection successful.")  # 接続が成功したことを表示
    # con.sendline("char-desc")  # キャラクタリスティックの記述子を取得

    # con.expect(["\[CON\]", "cba20002-224d-11e6-9fb8-0002a5d5c51b"])
    # # 特定のUUIDを持つ記述子が見つかるのを待つ
    # # cmd_handle = con.before.decode('utf-8').split('\n')[-1].split()[2].strip(',')  # 記述子のハンドルを取得
    # cmd_handle = (
    #     con.before.decode("utf-8").split("\n")[-1].split()[1].strip(",")
    # )  # chat-GPTはこっちが正しいと言ってる

    # con.sendline("char-write-req 13 0100")  # 通知を有効にする

    # con.expect("\[LE\]>")  # コマンド完了を待つ
    # con.sendline("quit")  # gatttoolを終了

    [mac, dev_type, act] = device
    # print 'Start to control'

    con = pexpect.spawn('gatttool -b ' + mac + ' -t random -I')
    con.expect('\[LE\]>')
    print('Preparing to connect.')
    retry = 2
    index = 0
    while retry > 0 and 0 == index:
        con.sendline('connect')
        # To compatible with different Bluez versions
        index = con.expect(
            ['Error', '\[CON\]', 'Connection successful.*\[LE\]>'])
        retry -= 1
        # if retry == 0:
        #     return False
    if 0 == index:
        print('Connection error.')
        return
    print('Connection successful.')
    con.sendline('char-desc')
    con.expect(['\[CON\]', 'cba20002-224d-11e6-9fb8-0002a5d5c51b'])
    cmd_handle = con.before.decode('utf-8').split('\n')[-1].split()[2].strip(',')
    con.sendline('char-write-req 13 0100')
    con.expect('\[LE\]>')

    if act == 'Turn On':
        con.sendline('char-write-cmd ' + cmd_handle + ' 570101')
    elif act == 'Turn Off':
        con.sendline('char-write-cmd ' + cmd_handle + ' 570102')
    elif act == 'Press':
        con.sendline('char-write-cmd ' + cmd_handle + ' 570100')
    # elif act == 'Down':
    #     con.sendline('char-write-cmd ' + cmd_handle + ' 570103')
    # elif act == 'Up':
    #     con.sendline('char-write-cmd ' + cmd_handle + ' 570104')

    con.expect('\[LE\]>')
    con.sendline('quit')
    print("test")
    return True
