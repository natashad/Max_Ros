import argparse
import math

from pythonosc import dispatcher
from pythonosc import osc_server

# def print_volume_handler(unused_addr, args, volume):
#   print("[{0}] ~ {1}".format(args[0], volume))

# def print_compute_handler(unused_addr, args, volume):
#   try:
#     print("[{0}] ~ {1}".format(args[0], args[1](volume)))
#   except ValueError: pass

def print_anything(unused_addr, args, param):
  print("[{0}] ~ {1}".format(args[0], param))

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--ip",
      default="127.0.0.1", help="The ip to listen on")
  parser.add_argument("--port",
      type=int, default=5005, help="The port to listen on")
  args = parser.parse_args()

  dispatcher = dispatcher.Dispatcher()
  dispatcher.map("/debug", print)
  dispatcher.map("/transport/timesig", print_anything, "TimeSignature")
  dispatcher.map("/transport/tempo", print_anything, "Tempo")
  dispatcher.map("/transport/resolution", print_anything, "Resolution")
  dispatcher.map("/transport/units", print_anything, "Units")
  dispatcher.map("/transport/beat", print_anything, "Beat")
  dispatcher.map("/transport/bar", print_anything, "Bar")

  # dispatcher.map("/volume", print_volume_handler, "Volume")
  # dispatcher.map("/logvolume", print_compute_handler, "Log volume", math.log)

  server = osc_server.ThreadingOSCUDPServer(
      (args.ip, args.port), dispatcher)
  print("Serving on {}".format(server.server_address))
  server.serve_forever()