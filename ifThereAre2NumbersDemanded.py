
import random
# θ(n) = nlog2(n)
def qsort(L):
    if len(L) <= 1:return L
    return qsort([it for it in L[1:] if it < L[0]])+L[0:1]+\
           qsort([ge for ge in L[1:] if ge >= L[0]])
# θ(n) = log2(n)
def find(L,num):
    count =1
    if L[-1]<num or L[0]>num:
        return False
    while True:
        length = len(L)
        mid = int(length/2)
        if length<=1:
            if L[0] ==num:
                return True
            return False
        if L[mid]==num:
            return True
        elif L[mid]<num:
            L = L[mid:]
        else:
            L = L[:mid]
# A:  a random list, x the target number to judge
# θ(n) = nlog2(n)+nlog2(n)
def ifExist(A,x):
    # get A sorted firstly
    A = qsort(A)
    for index,i in enumerate(A[:-1]):
        if find(A[(index+1):],x-i):
            return True
if __name__ =='__main__':
    test_list = [x for x in range(1000)]
    print(test_list)
    random.shuffle(test_list)
    print(ifExist(test_list,1998))
    # test succeeded！
    print(ifExist(test_list,1997))
    # test succeeded!