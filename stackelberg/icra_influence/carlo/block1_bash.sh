# Bash to run the randomized and balanced scenes automatically
# Order:
<<LatinSquare
I   R   H
R   H   I
H   I   R
LatinSquare

python3 intersection.py
python3 roundabout.py
python3 highway.py

python3 roundabout.py
python3 highway.py
python3 intersection.py

python3 highway.py
python3 intersection.py
python3 roundabout.py