import csv
import numpy as np

class Logdata:
    
    def __init__(self,fieldnames,ins_fieldnames):

        self.fieldnames = fieldnames
        self.ins_fieldnames = ins_fieldnames

    def create_log(self):
        with open('log_data/data.csv', 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames = self.fieldnames)
            csv_writer.writeheader()
        
    def add_data(self,i,attitude):
        self.i = i
        self.attitude = attitude
        with open('log_data/data.csv', 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames = self.fieldnames)
            att = {
                "i"     : self.i,
                "pitch" : self.attitude[0],
                "yaw"   : self.attitude[1],
                "roll"  : self.attitude[2],
                # "w"     : self.attitude[3],
                # "x"     : self.attitude[4],
                # "y"     : self.attitude[5],
                # "z"     : self.attitude[6],
                # "temp"  : self.attitude[7]
                }
            csv_writer.writerow(att)

    def create_ins_log(self, attitude):
        self.attitude = attitude
        with open('log_data/ins_data.csv', 'w') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames= self.ins_fieldnames)
            csv_writer.writeheader()

            ins_att = {
                "pitch"   : self.attitude[0],
                "yaw"    : self.attitude[1],
                "roll"  : self.attitude[2],
                }

            csv_writer.writerow(ins_att)

def create_fieldnames():
    fieldnames = ["i", "pitch", "yaw" , "roll"]
    ins_fieldnames = ["pitch", "yaw" , "roll"]
    return fieldnames, ins_fieldnames

