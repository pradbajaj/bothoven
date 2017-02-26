import serial
import time

#ser = serial.Serial("/dev/ttyUSB0", 9600)   # open serial port that Arduino is using
ser = serial.Serial("COM1", 9600)   # open serial port that Arduino is using


# 0 = 0
# 1 = 1
# 2 = 2
# 3 = 3
# 4 = 4
# 5 = 5
# 6 = 6
# 7 = 7
# 8 = 8
# 9 = 9
# 10 = :
# 11 = ;
# 12 = <
# 13 = =
# 14 = >
# 15 = ?
# 16 = @
# 17 = A
# 18 = B
# 19 = C
# 20 = D
# 21 = E
# 22 = F
# 23 = G
# 24 = H
# 25 = I
# 26 = J
# 27 = K
# 28 = L
# 29 = M
# 30 = N
# 31 = O
# 32 = P
# 33 = Q
# 34 = R
# 35 = S
# 36 = T
# 37 = U
# 38 = V
# 39 = W
# 40 = X
# 41 = Y
# 42 = Z
# 43 = [
# 44 = \
# 45 = ]
# 46 = ^
# 47 = _
# 48 = `
# 49 = a
# 50 = b
# end = e
# Sending Nodes 17, 33, 9, 6, 20

#Size would be 10 which is equal to ':'
ser.write(b'9')
time.sleep(0.1)

ser.write(b'<')
time.sleep(0.1)
ser.write(b'Q')
time.sleep(0.1)
ser.write(b'1')
time.sleep(0.1)
ser.write(b'A')
time.sleep(0.1)
ser.write(b'6')
time.sleep(0.1)
ser.write(b'H')
time.sleep(0.1)
ser.write(b'I')
time.sleep(0.1)
ser.write(b'N')
time.sleep(0.1)
ser.write(b'3')
time.sleep(0.1)

ser.write(b'e')
time.sleep(0.1)

ser.close()
