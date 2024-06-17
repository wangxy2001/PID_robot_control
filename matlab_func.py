import numpy as np
import cvxopt
def wrapToPi(x):
    #print(x.shape)
    for i in range(x.shape[0]):
        x[i]=(x[i]+np.pi)%(2*np.pi)-np.pi
        #if x[i] > np.pi:
        #    x[i] = x[i] - (np.floor(x[i] / (2 * np.pi)) + 1) * 2 * np.pi
        #elif x[i] < -np.pi:
        #    x[i] = x[i] + (np.floor(x[i] / (-2 * np.pi)) + 1) * 2 * np.pi
    #print("x",x)
    return x
def quadprog(H, f, L=None, k=None, Aeq=None, beq=None, lb=None, ub=None):
    """
    Input: Numpy arrays, the format follows MATLAB quadprog function: https://www.mathworks.com/help/optim/ug/quadprog.html
    Output: Numpy array of the solution
    """
    n_var = H.shape[1]

    P = cvxopt.matrix(H, tc='d')
    q = cvxopt.matrix(f, tc='d')

    if L is not None or k is not None:
        assert(k is not None and L is not None)
        if lb is not None:
            L = np.vstack([L, -np.eye(n_var)])
            k = np.vstack([k, -lb])

        if ub is not None:
            L = np.vstack([L, np.eye(n_var)])
            k = np.vstack([k, ub])

        L = cvxopt.matrix(L, tc='d')
        k = cvxopt.matrix(k, tc='d')

    if Aeq is not None or beq is not None:
        assert(Aeq is not None and beq is not None)
        Aeq = cvxopt.matrix(Aeq, tc='d')
        beq = cvxopt.matrix(beq, tc='d')

    sol = cvxopt.solvers.qp(P, q, L, k, Aeq, beq)

    return np.array(sol['x'])

"""
if __name__ == '__main__':
    H=np.array([[1,-1],[-1,2]])
    print(H)
    f=np.array([[-2],[-6]])
    print(f)
    L=np.array([[1,1],[-1,2],[2,1]])
    print(L)
    k=np.array([[2],[2],[3]])
    print(k)
    #min1/2*x‘Hx+f'x,s.t.Lx≤k,x≥0，算法中x为6x1，L3x6，k3x1 H6x6 f6x1
    res=quadprog(H, f, L,k)
    print(res)#[[0.6666667 ][1.33333334]]

"""
