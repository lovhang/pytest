import numpy as np
import math
rnd = np.random
rnd.seed(0)
S = 10
R = 4
T = 10
s = [i for i in range(1,S+1)] #road segment set
r = [i for i in range(1,R+1)] #treatment set include donothing
rs = [i for i in range(2,R+1)] #treatment set not include do nothing
t = [i for i in range(0,T+1)] #time step set
Qmax = 100 #maximum capacity
Qmin = 20 # minimum capacity
B = 5000000 #budget
Dm = 2.0 #initial deterioration rate
g = 3.0 #maximum number of treatment for each road segment
AADT = {} #AADT set
q0 = {} # initial capacity
d0 = {} # initial degradation

print(t)
print(s)
print(r)
for i in r:
    print(i)

e = {1:0.0, 2:10.0, 3:40.0, 4:80.0} #improvement value
D = {1:0.0, 2:2.0, 3:3.0, 4:5.0} #deterioration rate reduce value
c = {1:0.0, 2:5000.0, 3:170000.0, 4:285000.0} #cost
for i in s:
    AADT[i] = rnd.rand()*100
for i in s:
    q0[i] = rnd.rand()*(100-Qmin)+Qmin
for i in s:
    d0[i] = 20
print(AADT)
print(q0)
print(d0)

from docplex.mp.model import Model
mdl = Model('PMS')
x = mdl.binary_var_dict([(i,j,k) for i in s for j in r for k in t] , name='x')
q = mdl.continuous_var_dict([(i,j) for i in s for j in t], lb =Qmin,ub=Qmax, name='q')
d = mdl.continuous_var_dict([(i,j) for i in s for j in t], lb = 0.0, name='d')

mdl.minimize(mdl.sum(Qmax-q[i,j] for i in s for j in t))
mdl.add_constraints(q[i,k+1] <= q[i,k]-d[i,k] + mdl.sum(x[i,j,k]*e[j] for j in r) for i in s for k in range(0,T))
mdl.add_constraints(d[i,k+1] == d[i,k]-mdl.sum(x[i,j,k]*D[j] for j in r) for i in s for k in range(0,T))
#mdl.add_constraints(d[i,k] == Dm-mdl.sum(x[i,j,o]*D[j] for j in r for o in range(0,k)) for i in s for k in t)
mdl.add_constraint(mdl.sum(x[i,j,k]*c[j] for i in s for j in r for k in t)<=B)
#mdl.add_constraints(q[i,k] >= Qm for i in s for k in t)
mdl.add_constraints(mdl.sum(x[i,j,k] for j in rs for k in t)<=g for i in s)
mdl.add_constraints(mdl.sum(x[i,j,k] for j in r) == 1 for i in s for k in t)

#Initial state
mdl.add_constraints(q[i,0]==q0[i] for i in s)
mdl.add_constraints(d[i,0]==d0[i] for i in s)

mdl.print_information()
mdl.export_as_lp(path="D:\Python Cache\cplexPractise")
solution = mdl.solve(log_output=True)

if solution == None:
     print(mdl.solve_details)
else:
     print(solution)


file1 = open("output.txt","w")

file1.write("# of road segments: {}".format(S));file1.write(" \n")
file1.write("Planning period(year): {}".format(T));file1.write(" \n")
file1.write("Budget: {}".format(B));file1.write(" \n")
file1.write("minimum capacity: {}".format(Qmin));file1.write(" \n")
file1.write(" \n")
#deci = [(i,j,k) for i in s for j in r for k in t if x[(i,j,k)].solution_value>0.9]
#print(deci)
file1.write("S ")
for k in t:
    file1.write(" "+str(k)+" ")

file1.write(" \n")

for i in s:
    file1.write(str(i)+" ")
    for k in t:
        cer = False
        #file1.write(" 0 ")
        for j in r:
            #print(str(i)+"."+str(j)+"."+str(k)+": "+str(x[(i,j,k)].solution_value))
            if x[(i,j,k)].solution_value >0.9:
                if j == 1:
                    file1.write(" 0 ")
                elif j == 2:
                    file1.write(" P ")
                elif j == 3:
                    file1.write(" M ")
                elif j == 4:
                    file1.write(" R ")
    file1.write(" \n")

file1.write(" \n")

file1.write("{0:<5}".format("q"))

for k in t:
    file1.write("{0:<5}".format(k))

file1.write(" \n")
for i in s:
    file1.write("{0:<5}".format(i))
    for k in t:
        file1.write("{0:<5}".format(str(math.floor(q[(i,k)].solution_value))))
    file1.write(" \n")

file1.write(" \n")

file1.write("{0:<5}".format("d"))
for k in t:
    file1.write("{0:<5}".format(k))
file1.write(" \n")
for i in s:
    file1.write("{0:<5}".format(i))
    for k in t:
        file1.write("{0:<5}".format(str(math.floor(d[(i,k)].solution_value))))
    file1.write(" \n")

#i=1; j=2; k=3
#print(solution.get_value("x_"+str(i)+"_"+str(j)+"_"+str(k)))

file1.close()

mdl.end()

print("change test for git")

#sol = solution.get_value("d_1_1")
#print(sol)

