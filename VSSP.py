#!/usr/bin/env python
#coding:utf-8

import socket
import sys
import time
import struct
import math

class MeasurementHeader_t:
    def __init__(self, timestampHead, timestampTail, directionHead, directionTail, frame, h_field, line, spotHead, v_field, vInterlace):
        self.timestampHead = timestampHead
        self.timestampTail = timestampTail
        self.directionHead = directionHead
        self.directionTail = directionTail
        self.frame = frame
        self.h_field = h_field
        self.line = line
        self.spotHead = spotHead
        self.v_field = v_field
        self.vInterlace = vInterlace

# 極座標で表した3次元点群
class PolarPoints_t:
    def __init__(self, distances, mirrorAngles, motorAngles, swdr):
        self.distances = distances
        self.mirrorAngles = mirrorAngles
        self.motorAngles = motorAngles
        self.swdr = swdr

    def ToXYZPoints(self):
        # 直交座標変換を行う
        if (self.swdr):
            # ミラーの振り方向が水平
            x, y, z = zip(*[
                (
                    distance * math.cos(mirrorAngle) * math.cos(motorAngle),
                    distance * math.sin(mirrorAngle),
                    distance * math.cos(mirrorAngle) * math.sin(motorAngle)
                )
                for distance_list, mirrorAngle, motorAngle in zip(self.distances, self.mirrorAngles, self.motorAngles)
                for distance in distance_list
            ])
        else:
            # ミラーの振り方向が垂直
            x, y, z = zip(*[
                (
                    distance * math.cos(mirrorAngle) * math.cos(motorAngle),
                    distance * math.cos(mirrorAngle) * math.sin(motorAngle),
                    distance * math.sin(mirrorAngle)
                )
                for distance_list, mirrorAngle, motorAngle in zip(self.distances, self.mirrorAngles, self.motorAngles)
                for distance in distance_list
            ])
        return XYZPoints_t(x, y, z)

# 直交座標で表した3次元点群
class XYZPoints_t:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# 引数の値を2進数表現したときの1の個数を数える
def bitCount(i):
    # この時点では各ビットの値は各ビットに1がいくつあるかという値になっている
    i = (i & 0x55555555) + ((i >> 1) & 0x55555555)
    # この時点では2ビットごとの値は2ビットごとに1がいくつあるかという値になっている(最大2(10))
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333)
    # この時点では4ビットごとの値は4ビットごとに1がいくつあるかという値になっている(最大4(100))
    i = (i & 0x0f0f0f0f) + ((i >> 4) & 0x0f0f0f0f)
    # この時点では8ビットごとの値は8ビットごとに1がいくつあるかという値になっている(最大8(1000))
    i = (i & 0x00ff00ff) + ((i >> 8) & 0x00ff00ff)
    # この時点では16ビットごとの値は16ビットごとに1がいくつあるかという値になっている(最大16(10000))
    i = (i & 0x0000ffff) + ((i >> 16) & 0x0000ffff)
    # この時点では32ビットごとの値は32ビットごとに1がいくつあるかという値になっている(最大32(100000))
    return i & 0x3f

class AuxiliaryHeader_t:
    def __init__(self, timestamp, dataType, dataCount, dataPeriodMs):
        self.timestamp = timestamp
        self.dataType = dataType
        self.dataCount = dataCount
        self.dataPeriodMs = dataPeriodMs
        self.dataTypeCount = bitCount(self.dataType)

    @staticmethod
    def headerSize():
        return 12

class AuxiliaryData_t:
    def __init__(self, auxiliaryHeader, receiveData):
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.compassX = 0
        self.compassY = 0
        self.compassZ = 0
        self.temperature = 0

        pos = 0

        if ((auxiliaryHeader.dataType & (1 << 31)) != 0):
            (self.gyroX,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 30)) != 0):
            (self.gyroY,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 29)) != 0):
            (self.gyroZ,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 28)) != 0):
            (self.accX,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 27)) != 0):
            (self.accY,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 26)) != 0):
            (self.accZ,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 25)) != 0):
            (self.compassX,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 24)) != 0):
            (self.compassY,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 23)) != 0):
            (self.compassZ,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4
        if ((auxiliaryHeader.dataType & (1 << 22)) != 0):
            (self.temperature,) = struct.unpack("<l", receiveData[pos:pos+4])
            pos += 4

    def __str__(self):
        return str(self.gyroX) + ", " + str(self.gyroY) + ", " + str(self.gyroZ) + ", " + str(self.accX) + ", " + str(self.accY) + ", " + str(self.accZ) + ", " + str(self.compassX) + ", " + str(self.compassY) + ", " + str(self.compassZ) + ", " + str(self.temperature)


class VSSP:
    def open(self, host, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.settimeout(3)
        self.client.connect((host, port))
        self.pastTimestamp = 0
        self.tblh = []
        self.tblv = []
        self.mirrorAngles = []
        self.swdr = False

    def close(self):
        self.client.close()

    def recvVSSPPacket(self):
        commonHeaderSize = 24
        commonHeader = self.client.recv(commonHeaderSize)
        while (commonHeaderSize - len(commonHeader) > 0):
            #commonHeader += self.client.recv(commonHeaderSize - len(commonHeader))
            currentReceiveData = self.client.recv(commonHeaderSize - len(commonHeader))
            commonHeader += currentReceiveData
        #print(commonHeader)
        (mark, type, status, headerByteSize, commandByteSize, receiveTimestamp, sendTimestamp) = struct.unpack("<4s4s4sHHLL", commonHeader)
        #print("{0} - {1} = {2}".format(commandByteSize, headerByteSize, commandByteSize - headerByteSize))
        recvSize = commandByteSize - headerByteSize
        receiveData = self.client.recv(recvSize)
        while ((recvSize - len(receiveData)) > 0):
            #receiveData += self.client.recv(recvSize - len(receiveData))
            currentReceiveData = self.client.recv(recvSize - len(receiveData))
            receiveData += currentReceiveData
        return (status.strip(), type, receiveData)

    def VSSPSET(self, param, value):
        sendData = "SET:{0}={1}\n".format(param, value)
        self.client.send(sendData.encode())
        (status, type, receiveData) = self.recvVSSPPacket()
        return (status == b'000')

    def VSSPGET(self, param):
        sendData = "GET:{0}\n".format(param)
        self.client.send(sendData.encode())
        (status, type, receiveData) = self.recvVSSPPacket()
        if (status == b'000'):
            # statusが正常であれば取得したパラメータを返す
            return receiveData.decode().split()[1]
        else:
            # 以上の場合はNoneを返す
            return None

    def InitialSetting(self, horizontalInterlace, verticalInterlace):
        self.StopMeasurement()
        self.SetHorizontalInterlace(horizontalInterlace)
        self.SetVerticalInterlace(verticalInterlace)
        self.GetTblh()
        self.GetTblv(verticalInterlace)
        swdr = self.VSSPGET("swdr")
        if (swdr != None):
            self.swdr = (swdr == "1")

    def GetTblh(self):
        data = self.VSSPGET("tblh")
        self.tblh = [int(x, 16) for x in data.split(',')]

    def GetTblv(self, verticalInterlace):
        self.tblv = []
        self.mirrorAngles = []
        for num in range(verticalInterlace):
            data = self.VSSPGET("tv" + format(num, '02d'))
            tblv = [int(x, 16) for x in data.split(',')]
            self.tblv.append(tblv)
            # mirrorAnglesはデータごとに変わらないのでここで計算しておく(motorAnglesはデータごとに変わるので都度計算)
            self.mirrorAngles.append([v * 2 * math.pi / 65535 for v in tblv])

    def SetHorizontalInterlace(self, horizontalInterlace):
        self.VSSPSET("_itl", "0,{0:02}".format(horizontalInterlace))

    def SetVerticalInterlace(self, verticalInterlace):
        self.VSSPSET("_itv", "0,{0:02}".format(verticalInterlace))

    def ReadVSSPMeasurementHeader(self, receiveData):
        (headerSize,) = struct.unpack("<H", receiveData[0:2])
        if headerSize == 24:
            (timestampHead, timestampTail, directionHead, directionTail, frame, h_field, line, spotHead, v_field, vInterlace, reserve) = struct.unpack("<LLhhBBHHBBH", receiveData[2:headerSize])
            #print("timestamp = {0},{8}({7}), frame = {1}, directionHead = {2}, v_field = {3}, h_field = {4}, line = {5}, vInterlace = {6}".format(timestampHead, frame, directionHead, v_field, h_field, line, vInterlace, timestampHead - self.pastTimestamp, timestampTail))
            self.pastTimestamp = timestampHead
            measurementHeader = MeasurementHeader_t(timestampHead, timestampTail, directionHead, directionTail, frame, h_field, line, spotHead, v_field, vInterlace)
        elif headerSize == 20:
            (timestampHead, timestampTail, directionHead, directionTail, frame, h_field, line, spotHead) = struct.unpack("<LLhhBBHH", receiveData[2:headerSize])
            #print("timestamp = {0},{6}({5}), frame = {1}, directionHead = {2}, h_field = {3}, line = {4}".format(timestampHead, frame, directionHead, h_field, line, timestampHead - self.pastTimestamp, timestampTail))
            self.pastTimestamp = timestampHead
            measurementHeader = MeasurementHeader_t(timestampHead, timestampTail, directionHead, directionTail, frame, h_field, line, spotHead, 0, 1)
        else:
            measurementHeader = None
        return (headerSize, measurementHeader)

    def ReadVSSPAuxiliaryHeader(self, receiveData):
        (headerSize,) = struct.unpack("<H", receiveData[0:2])
        if headerSize == AuxiliaryHeader_t.headerSize():
            (timestamp, dataType, dataCount, dataPeriod) = struct.unpack("<LLBB", receiveData[2:headerSize])
            auxiliaryHeader = AuxiliaryHeader_t(timestamp, dataType, dataCount, dataPeriod)
        else:
            auxiliaryHeader = None

        return (headerSize, auxiliaryHeader)

    def ReadVSSPEchoIndexArray(self, receiveData, offset):
        (EchoIndexArraySize,SpotNum) = struct.unpack("<HH", receiveData[offset:offset + 4])
        padding = (SpotNum + 1) % 2 * 2
        #print("EchoIndexArraySize = {0}, SpotNum = {1}, padding = {2}".format(EchoIndexArraySize,SpotNum, padding))
        EchoIndexArray = struct.unpack("<" + "H" * (SpotNum + 1), receiveData[offset + 4:offset + EchoIndexArraySize - padding])
        #print("EchoIndexArray = {0}".format(EchoIndexArray))
        return (EchoIndexArray, EchoIndexArraySize)

    def ReadVSSPMeasurementData(self, receiveData, offset, isEnableIntensity, EchoIndexArray):
        distances = []
        intensities = []

        #print("len = {0}, {1}".format(len(receiveData), receiveData))
        #print("echoIndex = {0}".format(EchoIndexArray))

        for i, echoIndex in enumerate(EchoIndexArray[0:len(EchoIndexArray) - 1]):
            echoSize = EchoIndexArray[i + 1] - EchoIndexArray[i]
            distance = []
            intensity = []
            for echo in range(echoSize):
                echoUnit = 4 if isEnableIntensity else 2
                index = offset + (echoIndex + echo) * echoUnit
                # タプルをリストに変換してdistanceに格納
                distance.append(*struct.unpack("<H", receiveData[index:index + 2]))
                if isEnableIntensity:
                    intensity.append(*struct.unpack("<H", receiveData[index + 2:index + 4]))
            distances.append(distance)
            if isEnableIntensity:
                intensities.append(intensity)

        #print("Distances[{0}] = {1}".format(len(distances), distances))
        #print("Intensities[{0}] = {1}".format(len(intensities), intensities))

        return (distances, intensities)

    # 距離データの取得を行う
    def ReceiveMeasurementData(self):
        (status, type, receiveData) = self.recvVSSPPacket()
        (measurementHeaderSize, measurementHeader) = self.ReadVSSPMeasurementHeader(receiveData)
        # 距離データ応答じゃ無い場合は無視する
        if measurementHeaderSize != 20 and measurementHeaderSize != 24:
            return
        (EchoIndexArray, EchoIndexArraySize) = self.ReadVSSPEchoIndexArray(receiveData, measurementHeaderSize)
        (distances, intensities) = self.ReadVSSPMeasurementData(receiveData, measurementHeaderSize + EchoIndexArraySize, self.isEnableIntensity, EchoIndexArray)
        #print("frame =", measurementHeader.frame, ", h_field = ", measurementHeader.h_field, ", line =", measurementHeader.line, ", dir =", measurementHeader.directionHead, ", lineCount =", self.lineCount, flush = True)

        #print("self.mirrorAngles[{0}] = {1}".format(len(self.mirrorAngles), self.mirrorAngles))
        # このラインのモータ方向角度を計算
        motorAngles = [(measurementHeader.directionHead + (measurementHeader.directionTail - measurementHeader.directionHead) * h / 65535) * 2 * math.pi / 65535 for h in self.tblh]
        # このフィールドのミラー方向角度を取得
        mirrorAngles = self.mirrorAngles[measurementHeader.v_field]

        return (measurementHeader, PolarPoints_t(distances, mirrorAngles, motorAngles, self.swdr), intensities)

    def StartMeasurement(self, isEnableIntensity):
        self.isEnableIntensity = isEnableIntensity
        if isEnableIntensity:
            self.client.send(b"DAT:ri=1\n")
        else:
            self.client.send(b"DAT:ro=1\n")

    def StopMeasurement(self):
        self.client.send(b"DAT:ro=0\n")
        type = ""
        while (type != b"DAT:"):
            (status, type, receiveData) = self.recvVSSPPacket()

    def ReceiveAuxiliaryData(self):
        (status, type, receiveData) = self.recvVSSPPacket()
        (auxiliaryHeaderSize, auxiliaryHeader) = self.ReadVSSPAuxiliaryHeader(receiveData)
        if auxiliaryHeader != None:
            auxiliaryDatas = []
            pos = auxiliaryHeader.headerSize()
            for i in range(auxiliaryHeader.dataCount):
                auxiliaryDatas.append(AuxiliaryData_t(auxiliaryHeader, receiveData[pos:]))
                pos += auxiliaryHeader.dataTypeCount * 4
                """
                (a,b,c,d,e,f,g) = struct.unpack("<lllllll", receiveData[pos:pos+auxiliaryHeader.dataTypeCount*4])
                pos += auxiliaryHeader.dataTypeCount * 4
                print(a,b,c,d,e,f,g)
                """
            return auxiliaryDatas
        return None

    def StartAuxiliary(self):
        self.client.send(b"DAT:ax=1\n")
        self.recvVSSPPacket()

    def StopAuxiliary(self):
        self.client.send(b"DAT:ax=0\n")
        self.recvVSSPPacket()

if __name__ == "__main__":
    print("start")
    port = 10940
    IPAddress = "192.168.0.10"
    loopCount = 100
    # IPアドレスの指定がされていればそのIPアドレスに接続
    if len(sys.argv) >= 2:
        IPAddress = sys.argv[1]

    print("connect to " + IPAddress)
    YHT = VSSP()
    YHT.open(IPAddress, port)

    YHT.InitialSetting(horizontalInterlace = 4, verticalInterlace = 1)

    isEnableIntensity = True
    YHT.StartMeasurement(isEnableIntensity)

    for i in range(loopCount):
        tmp = YHT.ReceiveMeasurementData()
        if (tmp):
            (measurementHeader, polarPoints, intensities) = tmp
            print("frame={0}, hfield={1}, vfield={2}, line={3}, dist[0:10]={4}".format(measurementHeader.frame, measurementHeader.h_field, measurementHeader.v_field, measurementHeader.line, polarPoints.distances[0:10]))

    # stop
    YHT.StopMeasurement()
    YHT.close()
