/*
 * RungeKutta4.hh
 *
 *  created on: 22.04.2015
 *      author: rungger
 *
 *  customized by: M.Khaled on 21.10.2016
 */

#ifndef RUNGEKUTTA4_HH_
#define RUNGEKUTTA4_HH_

/* class: RungeKutta4 
 * Fixed step size ode solver implementing a Runge Kutta scheme of order 4 */
class OdeSolver {
private:
  const double dim_;
  const int nint_;
  const double h_;

public:
  OdeSolver(const int dim, const int nint, const double tau) : dim_(dim), nint_(nint), h_(tau/nint) { };

  template<class BaseObj, class RHS, class X, class U>
  inline void solve(BaseObj& obj, RHS rhs, X &x, U &u) {
		X k[4];

		for(size_t i=0; i<4; i++)
		    k[i].resize(x.size());

		X tmp(x.size());

		for(int t=0; t<nint_; t++) {
		    (obj.*rhs)(k[0],x,u);
			for(int i=0;i<dim_;i++)
				tmp[i]=x[i]+h_/2*k[0][i];

			(obj.*rhs)(k[1],tmp, u);
			for(int i=0;i<dim_;i++)
				tmp[i]=x[i]+h_/2*k[1][i];

			(obj.*rhs)(k[2],tmp, u);
			for(int i=0;i<dim_;i++)
				tmp[i]=x[i]+h_*k[2][i];

			(obj.*rhs)(k[3],tmp, u);
			for(int i=0; i<dim_; i++)
				x[i] = x[i] + (h_/6)*(k[0][i] + 2*k[1][i] + 2*k[2][i] + k[3][i]);
		}
	}
};

#endif /* RUNGEKUTTA4_HH_ */
