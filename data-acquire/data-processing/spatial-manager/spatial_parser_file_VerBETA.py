import sys
#import serial
from struct import *
import numpy as np



class SpatManag_parser:
    def __init__(self, file_name_in, file_name_out):
        
        self.in_file = open(file_name_in,'rb')
        self.out_file = open(file_name_out,'w')
        
        self.AN_PACKET_HEADER_SIZE = 5
        self.crc16_table = [
        
                0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273,
                0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528,
                0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886,
                0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf,
                0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5,
                0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2,
                0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8,
                0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691,
                0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f,
                0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64,
                0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
                ]
        self.total_msg = 0
        self.buf = []

    def decode_sys_st_pckt(self,data):
        sys_data = unpack('<HHffdddffffffffffffffff', bytes(data))
        accel = sys_data[10:13]
        gyro_itg = sys_data[17:20]
        gyro2 = sys_data[20:22]
        magn = sys_data[7:10]
        self.out_file.write(",".join(map(str,sys_data)))
        self.out_file.write('\n')
        # print(" Accel: {:.3f} {:.3f} {:.3f}\n Gyro_itg: {:.3f} {:.3f} {:.3f}\n Magn: {:.3f} {:.3f} {:.3f}\n Gyro2: {:.3f} {:.3f}\n\n".format(*accel, *gyro_itg, *magn, *gyro2))
    
    
    def decode_adxr_pckt(self,data):
        adxr_data = unpack('<ff', bytes(data))
        self.out_file.write(",".join(map(str,adxr_data)))
        self.out_file.write('\n')
        print("Gyro ADXRS {:.3f} Temp: {:.3f}".format(adxr_data[0], adxr_data[1]))
        
    def decode_mimu_pckt(self,data):
        mimu_data = unpack('<BLffffffffff', bytes(data))
        
        imuID = mimu_data[0]
        imuTime = mimu_data[1]
        accel = mimu_data[2:4]
        gyro = mimu_data[5:7]
        magn = mimu_data[8:10]
        temp = mimu_data[11]
        self.out_file.write(",".join(map(str,mimu_data)))
        self.out_file.write('\n')
        
    def buffer_add_data(self, buf, data):
        pass
    
    def calculate_header_lrc(self, data):
        return (((data[0] + data[1] + data[2] + data[3]) ^ 0xFF) + 1) & 0xFF;
    
    
    def calculate_crc16(self, data, poly=0x1021):
        #CRC-16-CITT poly, the CRC sheme used by ymodem protocol
        poly = 0x1021
        #16bit operation register, initialized to zeros
        crc = 0xFFFF
    
        for byte in data:
            crc = ((crc << 8) ^ self.crc16_table[(crc >> 8) ^ byte]) & 0xFFFF;
        return crc
    
    def decoder(self, buf, buffer_length):
        # global total_msg
        crc_errors = 0
        decode_iterator = 0
        length = 0
        crc = 0
        while (decode_iterator + self.AN_PACKET_HEADER_SIZE <= buffer_length):
            header_lrc = self.buf[decode_iterator]
            decode_iterator += 1
            # print(header_lrc, calculate_header_lrc(buf[decode_iterator:]))
            # print('buffer')
            # print(decode_iterator,buf[decode_iterator:])
            if (header_lrc == self.calculate_header_lrc(self.buf[decode_iterator:decode_iterator+self.AN_PACKET_HEADER_SIZE])):
                id = self.buf[decode_iterator]
                decode_iterator += 1
                length = self.buf[decode_iterator]
                decode_iterator += 1
                crc = self.buf[decode_iterator] 
                decode_iterator += 1
                crc |= self.buf[decode_iterator] << 8
                decode_iterator += 1
                if (decode_iterator + length > buffer_length):
                    decode_iterator -= self.AN_PACKET_HEADER_SIZE
                    break
    
                data = self.buf[decode_iterator:decode_iterator+length]
                # print(id, length, hex(crc), hex(calculate_crc16(buf[decode_iterator:decode_iterator+length])))
                if (crc == self.calculate_crc16(data)):
                    self.total_msg += 1
                    # print('CRC correcto')
                    # print(data)
                    if id == 20:
                        # decode_sys_st_pckt(data)
                        pass
                    elif id == 200:
                        self.decode_adxr_pckt(data)
                    elif id == 201:
                        self.decode_mimu_pckt(data)               
                    decode_iterator += length
    
                    continue
                else:
                    print('CRC incorrecto!!!')
                    # print(id, length, hex(crc), hex(calculate_crc16(buf[decode_iterator:decode_iterator+length])))
                    decode_iterator -= (self.AN_PACKET_HEADER_SIZE - 1)
                    crc_errors += 1
            else:
                print('LRC incorrecto!!!!!!!')
                print(hex(header_lrc), hex(self.calculate_header_lrc(self.buf[decode_iterator:decode_iterator+self.AN_PACKET_HEADER_SIZE])))
        #buf = buf[decode_iterator:]
        print('Good',decode_iterator, len(self.buf), self.total_msg)
        return self.buf
    
    def write_file(self):
        self.buf = self.in_file.read()
        self.buf = self.decoder(self.buf, len(self.buf))
        self.in_file.close()
        
    

if __name__ == "__main__":
    print("Enter the Spatial Manager file name:")
    spatial_file = input()
    print("Enter the name for the csv file:")
    csv_file = input()
    parseData = SpatManag_parser('../data-acquire/data-processing/spatial-manager/files/' + spatial_file + '.anpp', '../data-acquire/data-processing/post-processing/files/' + csv_file +'.csv')
    # parseData = SpatManag_parser('files/' + spatial_file + '.anpp', '../post-processing/files/' + csv_file +'.csv')
    parseData.write_file()
    
    # 'SpatialLog_21-04-21_14-11-10.anpp'