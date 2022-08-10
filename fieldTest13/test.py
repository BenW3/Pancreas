from tokenize import maybe
import methods
name = "testfile.csv"
methods.logDataInit(name)

mylist = [0.0, 1.1, 2.2, 3.3]
mylist2 = ["1,2,3", "4,5,6" , "7,8,9", "10,11,12"]
data = []
i = 0
while i < len(mylist):
    data.append(str(mylist[i])+","+str(mylist2[i]))
    i += 1
methods.logDataUpdate(data, name)


