import os
import csv
def main():
    print('cane')
    header= ["a","b","c","d"]
    datas = [1,2,3,4]
    data = open('Revolution_Results.csv',"w+")
    writer = csv.writer(data)
    writer.writerow(header)
    for i in range(5):
        writer.writerow(datas)
        
    data.close()

if __name__ == "__main__":
    main()