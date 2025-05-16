import pickle
import argparse
print("RAN")
#options are 0 for baseline and 1 for ours
parser = argparse.ArgumentParser()
parser.add_argument('--algo', type=int, default=0)
parser.add_argument('--user', type=int, default=0)
args = parser.parse_args()

savename = "data/user" + str(args.user) + "/run" + str(args.algo)
input_data = open(savename, 'rb')

# get run data
try:
    saved_data = pickle.load(input_data)
except EOFError:
    True
print(len(saved_data))
