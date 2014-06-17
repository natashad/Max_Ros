"""
This program sends 10 random values between 0.0 and 1.0 to the /filter address,
waiting for 1 seconds between each value.
"""
import argparse
import random
import time

from pythonosc import osc_message_builder
from pythonosc import udp_client

_time_sig_num = 4
_time_sig_denom = 4
_tempo = float(160)
_resolution = float(480)
_bar = 0

addrs = ['/transport/timesig', '/transport/tempo', '/transport/resolution', '/transport/units', '/transport/beat', '/transport/bar']

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--ip", default="127.0.0.1",
      help="The ip of the OSC server")
  parser.add_argument("--port", type=int, default=5005,
      help="The port the OSC server is listening on")
  args = parser.parse_args()

  client = udp_client.UDPClient(args.ip, args.port)

  initTime = time.time()
  while True:
    timeDeltaMins = (time.time() - initTime)/60.0

    # Change if not using 4/4 time.
    _beat = int((_tempo * timeDeltaMins)%_time_sig_num)
    _bar = int(float(_tempo * timeDeltaMins)/_time_sig_denom)

    # TODO: Calculate
    _units = 260
    _time_sig = str(_time_sig_num) + " " + str(_time_sig_denom)


    addr_params = [_time_sig, _tempo, _resolution, _units, _beat, _bar]

    for ind, addr in enumerate(addrs):
      msg = osc_message_builder.OscMessageBuilder(address = addr)
      msg.add_arg(addr_params[ind])
      msg = msg.build()
      client.send(msg)
      print("Sending message")


    # NOTE DATA MESSAGE
    msg = osc_message_builder.OscMessageBuilder(address = "/clip/notes")
    msg.add_arg("Kick")
    msg.add_arg(144)
    msg.add_arg("fake_name")
    msg.add_arg(10)
    msg.add_arg(2)

    # note 1
    msg.add_arg(1)
    msg.add_arg(0.5)
    msg.add_arg(0.5)
    msg.add_arg(100)
    msg.add_arg(0)

    # note 2
    msg.add_arg(10)
    msg.add_arg(4.75)
    msg.add_arg(0.5)
    msg.add_arg(100)
    msg.add_arg(0)

    msg = msg.build()
    client.send(msg)
    print("Sending message")


    time.sleep(0.5)
