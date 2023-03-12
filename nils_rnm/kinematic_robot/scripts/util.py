# Utility functions used throughout the package

import csv
import os

def get_list_from_csv(dir_name, file_name):
    ''' Get a list of lists from a csv file
        dir_name should be relative to the location of util.py
    '''

    output  = []
    with open((os.path.join(os.path.dirname(__file__), dir_name, file_name)), newline="") as csv_file:
        reader  = csv.reader(csv_file)
        for row in reader:
            row_float = [float(entry) for entry in row]
            output.append(row_float)

    return output
