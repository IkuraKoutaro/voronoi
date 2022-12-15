dic = {}
key = ['1', '2', '3']

a = [1,2,3]
b = [4,5,6]


for k in key:
    dic[k] = []

for i in range(3):
    dic[key[0]].append(a)
    dic[key[1]].append(b)
    dic[key[2]].append(a)
    print(dic)

print(dic[key[0]])

print(' ')

for i in dic:
    num1, num2, num3 = dic[i]
    print(num1)