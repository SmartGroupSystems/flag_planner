#include "bspline_race/bspline_race.h"

namespace FLAG_Race

{
    UniformBspline::UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        initUniformBspline(p, n,beta, D, s_ini, s_ter); 
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                 const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter)
    {
        p_ = p; 
        n_ = n-1;
        beta_ = beta;
        D_ =D;
        m_ = p_+n_+1;
        u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 共m+1个
        control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
        for(int i = 0; i<=m_; i++)
        {
            u_(i) = i;
        }
        s_ini_ = s_ini;
        s_ter_ = s_ter;
        setIniTerMatrix();
        getAvailableSrange();
        getAvailableTrange();
        getInterval();
    }

    void UniformBspline::setIniTerMatrix()
    {
        A_ini.resize(3,3);
        A_ter.resize(3,3);
        A_ini << 1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
        A_ter<<1.0/6, 2.0/3, 1.0/6,
                        -1.0/2*beta_, 0.0*beta_, 1.0/2*beta_,
                        1.0*beta_*beta_,-2.0*beta_*beta_,1.0*beta_*beta_;
    }

    void UniformBspline::setControlPoints(const Eigen::MatrixXd &ctrl_points)
    {
        control_points_ = ctrl_points;
    }
    
    Eigen::MatrixXd UniformBspline::getTrajectory(const Eigen::VectorXd &t)
    {
        double u_probe;
        int t_size = t.size();
        Eigen::MatrixXd trajectory(t_size,D_);
        for (size_t i = 0; i < t_size; i++)
        {
            //map t(i) to uniform knot vector
            u_probe = t(i) * beta_ + u_(p_);
            trajectory.row(i) = singleDeboor(u_probe); 
        }
        return trajectory;
    }

     Eigen::Vector2d UniformBspline::singleDeboor(const double &u_probe)//the deboor's algorithm
     {  
        //bound the u_probe
        double u_probe_;
        int k;
        u_probe_ = min(max( u_(p_) , u_probe), u_(m_-p_));
        k = p_;
        while(true)
        {
            if(u_(k+1)>=u_probe_)
                break;
            k = k+1;
        }
        // t(t_ctt) is maped to knot segment u_k ~ u_{k+1}
        // there are at most p+1 basis functions N_k-p,p(u), N_k-p+1,p(u),..., N_k,p(u) non-zero on knot span [uk,uk+1)
        //the effective control points are P_k-p ~ P_k
        // since MATLAB index start from 1 instead of 0
        // The effective control points are
        double alpha;
        Eigen::MatrixXd d(p_+1,2);
        d = control_points_.block(k-p_,0,p_+1,2);// c++这里是从0行0列开始
        for (size_t i = 0; i < p_; i++)
        {
            for (size_t j = p_; j > i; j--)
            {
                alpha = (u_probe_ - u_(j+k-p_)) /(u_(j+k-i) - u_(j+k-p_)); 
                d.row(j) = (1 - alpha)*d.row(j-1) + alpha*d.row(j);
            }          
        }

            Eigen::Vector2d value;
            value = d.row(p_);
            return value;
     }

    void UniformBspline::getAvailableSrange()
    {
        s_range = {u_(p_),u_(m_-p_)};
    }

    void UniformBspline::getAvailableTrange()
    {
        t_range = {0/beta_, (u_(m_-p_)-u_(p_))/beta_};
    }

    void UniformBspline::getInterval()
    {
        interval_ = (u_(1) - u_(0))/beta_;
    }

    void UniformBspline::getT(const int &trajSampleRate)
    {
        time_.resize((t_range(1)-t_range(0))*trajSampleRate+1);
        
        for (size_t i = 0; i < time_.size(); i++)
        {
            time_(i) = t_range(0) + i*(1.0/trajSampleRate);
        }
    }

    UniformBspline UniformBspline::getDerivative()
    {     
            UniformBspline spline(p_,n_,beta_,D_,s_ini_,s_ter_);
            spline.p_ = spline.p_ -1;
            spline.m_ = spline.p_ +spline.n_ +1;
            spline.u_.resize(u_.size()-2);
            spline.u_ = u_.segment(1,m_-1);//从第2个元素开始的m-1个元素
            spline.control_points_.resize(control_points_.rows()-1,D_);
            for (size_t i = 0; i < spline.control_points_.rows(); i++)
            {
                spline.control_points_.row(i) = spline.beta_*(control_points_.row(i+1) - control_points_.row(i));
            } 
            spline.time_ = time_;
            return spline;
    }

    Eigen::VectorXd UniformBspline::getBoundConstraintb()
    {
        int nm = (n_+1)*D_;
        Eigen::VectorXd b= Eigen::VectorXd::Zero(nm);
        Eigen::MatrixXd tmp1(3,2);//前三个控制点的值
        Eigen::MatrixXd tmp2(3,2);//末尾三个控制点的值
        // solve Ax = b
        tmp1 = A_ini.colPivHouseholderQr().solve(s_ini_);
        tmp2 = A_ter.colPivHouseholderQr().solve(s_ter_);
         for (size_t j = 0; j< D_; j++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                b(i+j*(n_+1)) = tmp1(i,j);
                b((j+1)*(n_+1)-i-1) = tmp2(3-i-1,j);
            }      
        }    
        return b;   
    }

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/


    bspline_optimizer::bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p)
    {
        path_.clear();
        path_ = path;
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = path.size() + 2*p_order_ -2;
    }
    bspline_optimizer::bspline_optimizer(const int&Dim,  const int&p, const double &dist)
    {
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = 2*p_order_+floor(dist/1.0);
    }
    bspline_optimizer::~bspline_optimizer(){}
    
    void bspline_optimizer::setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                                                    const double safe_dist)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            lambda3_ = lambda3;
            safe_distance_ = safe_dist;
    }
    void bspline_optimizer::setVelAcc(const double vel, const double acc)
    {
            max_vel_ = vel;
            max_acc_ = acc;
    }
  void bspline_optimizer::setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            max_vel_ = vel;
            max_acc_ = acc;
    }
    void bspline_optimizer::setEsdfMap(const Eigen::MatrixXd &esdf_map)
    {
        if(esdf_map.size()==0)
        {
          esdf_map_.setZero(1000,1000); 
          cout<<" esdf map is empty!!!!!!"<<endl;
        }
        else
        esdf_map_ = esdf_map;
    }

    void bspline_optimizer::setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                                                    const double &start_x, const double &start_y)
    {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        map_resolution_ = map_resolution;
        startPoint_x = start_x;
        startPoint_y = start_y;
    }

    void bspline_optimizer::setSplineParam(const UniformBspline &u)
    {
        u_ = u;
        bspline_interval_  = u.interval_;
        beta_ = u.beta_;
        control_points_.resize(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u_.getBoundConstraintb();
        
        for (size_t i = 0; i < Dim_; i++)
        {
                for (size_t j = 0; j < p_order_; j++)
                {
                     control_points_(j,i) = beq_bound(i*cps_num_+j);
                     control_points_((1)*cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);//BUG!!!!
                }
        }
            
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
            control_points_.row(i+p_order_) = path_[i+1];

        }
    }
        void bspline_optimizer::initialControlPoints(UniformBspline u)
        {
        control_points_.setZero(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u.getBoundConstraintb();
        for (size_t i = 0; i < Dim_; i++)
        {
        for (size_t j = 0; j < p_order_; j++)
        {
        control_points_(j,i) = beq_bound(i*cps_num_+j);
        control_points_(cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
        }
        }
        int insert_num = cps_num_-2*p_order_;
        Eigen::Vector2d start_pos = control_points_.row(p_order_-1);
        Eigen::Vector2d end_pos = control_points_.row(cps_num_-p_order_);
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
        control_points_(i+p_order_,0) = start_pos(0)+(end_pos(0)-start_pos(0))/(insert_num+1)*(i+1) ;
        control_points_(i+p_order_,1) = start_pos(1)+(end_pos(1)-start_pos(1))/(insert_num+1)*(i+1) ;
        }
       // cout<<control_points_<<endl;
        }
    void bspline_optimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector2d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector2d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }
  void bspline_optimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
   {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector2d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }
    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector2d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }

    }
    }
    
    void bspline_optimizer::calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double  dist;
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d tmp_vel;

        for (int i = p_order_; i < q.cols()-p_order_; i++) 
        {
            if(q(0,i)<-40||q(0,i)>40||q(1,i)<-40||q(1,i)>40)
            {
                cout<<"unreasonable control points"<<endl;
            }
            else{
                dist = calcDistance(q.col(i));
                dist_grad = calcGrad(q.col(i));
                if (dist_grad.norm() > 1e-4) dist_grad.normalize();
                if (dist < safe_distance_) 
                {
                    cost += pow(dist - safe_distance_, 2);
                    gradient.col(i) += 2.0 * (dist - safe_distance_) * dist_grad;     
                }
            }

        }   

    }
    double bspline_optimizer::calcDistance(const Eigen::MatrixXd &q)
    {
        double dist;
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);
        double dists[2][2];
        getSurroundDistance(sur_pts, dists);
        interpolateBilinearDist(dists, diff, dist);
        return dist;
    }

    void bspline_optimizer::interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, 
                                                                                                                                                                double& dist)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        dist = lerp(ny0,ny1,ty);
    }
    Eigen::Vector2d bspline_optimizer::calcGrad(const Eigen::MatrixXd &q)
    {
        Eigen::Vector2d p(q(0,0),q(1,0));//存入两个控制点
        Eigen::Vector2d dist_grad;
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];//4个邻居点
        getSurroundPts(p,sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist_grad);

        return dist_grad;
    }

    void bspline_optimizer::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff)
    {
        double dist_x = pos(0) - startPoint_x;
        double dist_y = startPoint_y - pos(1);
        diff(0) = fmod(dist_x,map_resolution_);
        diff(1) = fmod(dist_y,map_resolution_);

        Eigen::Vector2d curr_index;//用这个索引找到最左上角的点，并记为 p(0,0);
        curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {       
                Eigen::Vector2d tmp_index(curr_index(0)+i,curr_index(1)+j);
                pts[i][j] = tmp_index;
            }
        }
    }
    void bspline_optimizer::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < 2; j++)
            {
                Eigen::Vector2d tmp_index = pts[i][j];
                dists[i][j] = esdf_map_(tmp_index(0),tmp_index(1));   
            }
        }
    }
    void bspline_optimizer::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad)
    {
        //二线性插值
        double c00 = values[0][0];
        double c01 = values[0][1];
        double c10 = values[1][0];
        double c11 = values[1][1];
        double tx = diff(0);
        double ty = diff(1);
        
        double nx0 = lerp(c00,c10,ty);
        double nx1 = lerp(c01,c11,ty);
        double ny0 = lerp(c00,c01,tx);
        double ny1 = lerp(c10,c11,tx);

        grad(0) = (nx1- nx0)/map_resolution_;
        grad(1) = (ny0- ny1)/map_resolution_;
    }

    void bspline_optimizer::combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_length, f_distance, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_feasibility_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_distance_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  = f_feasibility =  f_distance = 0.0;
        // cout<< control_points.transpose()<<endl;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
    // cout<<"====================calcSmoothnessCost"<<endl;
        calcFeasibilityCost(control_points,f_feasibility,g_feasibility_);
    // cout<<"====================calcFeasibilityCost"<<endl;
        calcEsdfCost(control_points,f_distance,g_distance_);
    // cout<<"====================calcEsdfCost"<<endl;

        f_combine = lambda1_ * f_smoothness + lambda2_*f_feasibility + lambda3_*f_distance;
        grad2D = lambda1_*g_smoothness_ + lambda2_ * g_feasibility_ +lambda3_ * g_distance_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }
    void bspline_optimizer::combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        //cout<<control_points<<endl;
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad2D; 
        grad2D.resize(Dim_,cps_num_);
        grad2D.setZero(Dim_,cps_num_);//初始化梯度矩阵

        double f_smoothness, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  =  0.0;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
        grad2D = lambda1_*g_smoothness_ ;
        grad = grad2D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }
    double bspline_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCost(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }
    double bspline_optimizer::costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCostSmooth(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }


     void bspline_optimizer::optimize()
    {
            /* initialize solver */
            // cout << "/* initialize solver */"<<endl;
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num); 
            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
        {
            for (size_t i = 0; i < Dim_; i++)
            {
                control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
            } 
        }
        cout<< "optimize successfully~"<<endl;
            // cout << "iner:\n"<<control_points_<<endl;
            // cout<<"iter num :"<<iter_num_<<endl;
    }
 void bspline_optimizer::optimizesmooth()
    {
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunctionSmooth,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 5.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            cout<< "optimize successfully~"<<endl;
    }

    void bspline_optimizer::optimize_withoutesdf()
    {
           double intial_lambda3 = lambda3_;
            lambda3_  = 0;
            /* initialize solver */
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(200);
            opt.set_maxtime(0.02);
            opt.set_xtol_rel(1e-5);
            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num);

            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);
        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);    
        }
        catch(std::exception &e)
         {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

          for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
                } 
            }
            lambda3_  = intial_lambda3;
    }



/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
   //只用于测试
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        setParam(nh);
        TrajPlanning(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::setParam(ros::NodeHandle &nh)
    {
        last_time_ = ros::Time::now().toSec();
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/TrajSampleRate", TrajSampleRate, -1);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/goal_x", goal_x_, -1.0);
        nh.param("planning/goal_y", goal_y_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        nh.param("planning/lambda3",lambda3_,-1.0);
        nh.param("planning/frame",frame_,std::string("odom"));
        nh.param("planning/map_resolution",map_resolution_,-1.0);
        nh.param("planning/start_x",start_x_,-1.0);
        nh.param("planning/start_y",start_y_,-1.0);
        nh.param("planning/start_x",startPoint_x,-1.0);
        nh.param("planning/start_y",startPoint_y,-1.0);
        nh.param("planning/safe_distance",safe_distance_,-1.0);
        nh.param("planning/esdf_collision",esdf_collision,2.0);
        nh.param("planning/dist_p",dist_p,-1.0);
        
        
        beta = max_vel_/dist_p;
        cout<< "beta is "<<beta<<endl;

        lambda3_saved = lambda3_;
    }

    void plan_manager::TrajPlanning(ros::NodeHandle &nh)
    {
        //goal_suber = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, & plan_manager::uav_goal_subCallback, this);
        fsm_suber = nh.subscribe<std_msgs::Int64>("/flag_detect",1,&plan_manager::fsm_subCallback,this);

        //订阅地图
        map_suber = nh.subscribe<std_msgs::Float64MultiArray>("/ESDFmsgs",1,&plan_manager::esdf_map_subCallback,this);
        
        //订阅路径
        path_suber = nh.subscribe<nav_msgs::Path>("/astar_node/grid_twist",1, &plan_manager::astar_subCallback,this);

        //订阅起止点
        waypoint_suber = nh.subscribe<nav_msgs::Path>("/waypoint",1, &plan_manager::smooth_subCallback,this);

        //发布轨迹
        Traj_puber = nh.advertise<bspline_race::BsplineTraj>("/bspline_traj", 10);
        Time_puber = nh.advertise<std_msgs::Float64>("/back_time", 10);

        //可视化执行的轨迹   
        Traj_vis = nh.advertise<nav_msgs::Path>("/traj_vis", 10);
        Traj_vis1 = nh.advertise<nav_msgs::Path>("/traj_smooth", 10);

        //可视化地图
        Map_puber = nh.advertise<visualization_msgs::Marker>("/esdfmap_slice", 10);

        traj_smooth = nh.advertise<bspline_race::BsplineTraj>("bspline_smooth",10);

        col_check = nh.advertise<std_msgs::Bool>("/col_check",10);

        //同步飞机位置速度消息
        subscriber_vel = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh,"/mavros/local_position/velocity_local_orb",10);
        subscriber_pos = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh,"/mavros/local_position/pose_orb",10);
        subscriber_acc = new message_filters::Subscriber<sensor_msgs::Imu>(nh,"/mavros/imu/data",10);
        sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *subscriber_vel, *subscriber_pos, *subscriber_acc);  
        sync->registerCallback(boost::bind(&plan_manager::current_state_callback,this, _1, _2, _3));

        //获取当前aim
        // fullaim_suber = nh.subscribe<mavros_msgs::PositionTarget>("/mavbs/setpoint_raw/local",1,&plan_manager::fullaim_callback,this);
        arrived_suber = nh.subscribe<std_msgs::Int64>("/astar_node/target_arrived",10,&plan_manager::arrive_callback,this);

    }
void plan_manager::arrive_callback(const std_msgs::Int64::ConstPtr & msg)
{
    if(msg->data>0)
    {
        lambda3_ = lambda3_saved;
    }
    // cout<<lambda3_<<endl;
}
void plan_manager::fsm_subCallback(const std_msgs::Int64::ConstPtr & msg)
{
    if(msg->data == 3)
        enable_flag = true;
    else 
        enable_flag = false;
}

// void plan_manager::fullaim_callback(const mavros_msgs::PositionTarget::ConstPtr & aim_msg)
// {
//     current_aim[0] = aim_msg->position.x;
//     current_aim[1] = aim_msg->position.y;
//     current_vel[0] = aim_msg->velocity.x;
//     current_vel[1] = aim_msg->velocity.y;
//     current_acc[0] = aim_msg->acceleration_or_force.x;
//     current_acc[1] = aim_msg->acceleration_or_force.y;
//     current_seq = (int)aim_msg->yaw;
//     // cout << "当前位置：\n" << current_aim <<endl;
    
//     back_time_= ros::Time::now().toSec();
// }

void plan_manager::current_state_callback(const geometry_msgs::TwistStampedConstPtr & vel_msg,
                                          const geometry_msgs::PoseStampedConstPtr &pos_msg,
                                          const sensor_msgs::ImuConstPtr &imu_msg)
    {
        // current_pos<< pos_msg->pose.position.x,pos_msg->pose.position.y;
        // current_vel<< vel_msg->twist.linear.x,vel_msg->twist.linear.y;
        // current_acc<< imu_msg->linear_acceleration.x,-imu_msg->linear_acceleration.y;// BUG!!
    }

    void plan_manager::uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
    {
        terminal_state(0,0) = goal_msg->pose.position.x;
        terminal_state(0,1) = goal_msg->pose.position.y; 
    }

    void plan_manager::esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg)
    {
        cout<< "get grid map"<<endl;
        get_map = true;
        map_size_x = map_msg->data[0];
        map_size_y = map_msg->data[1];
        map_size = map_size_x*map_size_y;
        grid_map_.setZero(map_size_x,map_size_y);//grid map 初始化全0
        esdf_map_=100* Eigen::MatrixXd::Ones(map_size_x,map_size_y);//esdf map 初始化全100
        double *src,*dst;
        src=(double*)malloc(map_size*sizeof(double));
        dst=(double*)malloc(map_size*sizeof(double));

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                grid_map_(map_size_x-j-1,i) = map_msg->data[i*map_size_x+j+2];
            }
        }

        for (size_t i = 0; i < map_size_x; i++)
        {
            for (size_t j = 0; j < map_size_y; j++)
            {
                //把grid_map里的数据送入src指针
                *(src+i*esdf_map_.rows()+j) = grid_map_(i,j);
                
            }
        }

        computeEDT(dst,src,map_size_x,map_size_y);

        for (size_t i = 0; i < map_size_x; i++)
        {
            if( (int)sqrt(*(dst+i*esdf_map_.rows()))<-1000)break;
            else
            {
            for (size_t j = 0; j < map_size_y; j++)
            {
                esdf_map_(i,j) = (int)sqrt(*(dst+i*esdf_map_.rows()+j));
            }
            }
        }      
        map_slice_output(esdf_map_);
        // map_slice_output(grid_map_);
        free(dst);
        free(src);
        dst = NULL;
        src = NULL;

    }



    void plan_manager::astar_subCallback(const nav_msgs::PathConstPtr &path)
    { 
        if(!get_map) return;
        //get_map = false;
        astar_path_.clear();
        // get_path = true;
        //读取Astar
        Eigen::Vector2d tmp_point;
        double delta_t = 0.1;
        Eigen::Vector2d current_aim = Eigen::Vector2d::Zero();
        Eigen::Vector2d current_vel = Eigen::Vector2d::Zero();
        Eigen::Vector2d current_acc = Eigen::Vector2d::Zero();
        current_vel << path->poses[0].pose.position.x,path->poses[0].pose.position.y;
        current_seq = path->poses[0].pose.position.z;
        current_acc << path->poses[1].pose.position.x,path->poses[1].pose.position.y;
        current_aim << path->poses[2].pose.position.x,path->poses[2].pose.position.y;
        for (size_t i = 2; i < path->poses.size(); i++)
        {
            tmp_point<< path->poses[i].pose.position.x,path->poses[i].pose.position.y;
            //A star路径是正的
            astar_path_.push_back(tmp_point);
        }
        //读取首末位置
        Eigen::Vector2d start_point,end_point;
        start_point = *astar_path_.begin();
        end_point = *(astar_path_.end()-1);
        initial_state.resize(3,2);
        terminal_state.resize(3,2);
        std_msgs::Float64 msg_time;
        msg_time.data = back_time_;
        Time_puber.publish(msg_time);    
        initial_state << current_aim(0), current_aim(1),
                         current_vel(0), current_vel(1),
                         current_acc(0), current_acc(1);
        terminal_state << end_point(0), end_point(1),
		                    0.0, 0.0,
	                        0.0, 0.0;
        double now_time_  = ros::Time::now().toSec() ;
        double delta_time = now_time_ - last_time_;//|| last_endpoint != end_point
        if( first_rifine == true || delta_time > 0.2 || checkTrajCollision() == true )// 
        {
            last_time_ = now_time_;
            first_rifine = false;
            last_endpoint = end_point;
            //初始化优化类和 b样条轨迹类
            opt.reset(new bspline_optimizer(astar_path_,Dim_,p_order_));
            u.reset(new UniformBspline(p_order_,opt->cps_num_,beta,Dim_,initial_state,terminal_state));
            UniformBspline spline = *u;
            opt->setEsdfMap(esdf_map_) ;
            opt->setOptParam(lambda1_,lambda2_,lambda3_,safe_distance_);
            opt->setMapParam(origin_x_,origin_y_,map_resolution_,start_x_,start_y_);
            opt->setVelAcc(max_vel_,max_acc_);
            opt->setSplineParam(spline); 
            opt->optimize(); 
            // 计算轨迹
            cout <<"\033[46m--------------------grid_path - control_points_--------------------"<<endl;
            int i = 0;
            for(auto ptr : astar_path_)
            {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
            cout << Eigen::Vector2d(ptr - xxx).norm() <<endl;
            i++;
            }
            cout <<"----------------------------------------\033[0m"<<endl;
            auto old_ptr = (*astar_path_.begin());
            cout <<"\033[45m--------------------grid_path--------------------"<<endl;
            for(auto ptr : astar_path_)
            {
            cout << ptr <<endl;
            cout << "- - -"<<endl;
            cout << Eigen::Vector2d(ptr - old_ptr).norm() <<endl;
            cout << "- - -"<<endl;
            old_ptr = ptr;
            }
            cout <<"----------------------------------------\033[0m"<<endl;
            cout <<"\033[45m--------------------control_points_--------------------"<<endl;
            i = 0;
            for(auto ptr : astar_path_)
            {Eigen::Vector2d xxx(opt->control_points_(3+i,0),opt->control_points_(3+i,1));
            cout << xxx <<endl;
            cout << "- - -"<<endl;
            i++;
            }
            cout <<"----------------------------------------\033[0m"<<endl;
            u->setControlPoints(opt->control_points_);
            u->getT(TrajSampleRate);
            UniformBspline p = *u;
            UniformBspline v = p.getDerivative();
            UniformBspline a = v.getDerivative();

            //生成轨迹
            geometry_msgs::PoseStamped tmp_p,tmp_v,tmp_a;
            geometry_msgs::PoseStamped tmp_vis;
            p_ = p.getTrajectory(p.time_);
            v_ = v.getTrajectory(p.time_);
            a_ = a.getTrajectory(p.time_);
            traj.position.clear();
            traj.velocity.clear();
            traj.acceleration.clear();
            traj_vis.poses.clear();      
            for (size_t i = 0; i < p_.rows(); i++)
            {
                // int count = p_.rows()-i-1;
                int count = i;
                tmp_p.header.seq = count;
                tmp_v.header.seq = count;
                tmp_a.header.seq = count;
                tmp_p.pose.position.x   = p_(count,0);tmp_p.pose.position.y   = p_(count,1);tmp_p.pose.position.z   = 0;
                tmp_v.pose.position.x   = v_(count,0); tmp_v.pose.position.y  = v_(count,1);tmp_v.pose.position.z   = 0;
                tmp_a.pose.position.x   = a_(count,0); tmp_a.pose.position.y  = a_(count,1); tmp_a.pose.position.z  = 0;
                tmp_vis.pose.position.x = p_(count,0);tmp_vis.pose.position.y = p_(count,1);tmp_vis.pose.position.z = 0.5;
                traj.position.push_back(tmp_p) ;
                traj.velocity.push_back(tmp_v) ;
                traj.acceleration.push_back(tmp_a);
                traj_vis.poses.push_back(tmp_vis);
                traj.header.frame_id = frame_;
                traj_vis.header.frame_id = frame_;
            }
            Traj_vis.publish(traj_vis);
            Traj_vis1.publish(traj_vis1);

            //发布期望轨迹
            traj.current_seq = current_seq;
            Traj_puber.publish(traj);
        }
            
    }

/*****************************************
 * smooth_path
 * 输入 :nav_msgs::Path
 * 输出:同astar
 * 
 * *************************************/
void plan_manager::smooth_subCallback(const nav_msgs::PathConstPtr &msg)
{

}

void plan_manager::map_slice_output(const Eigen::MatrixXd &esdf_matrix)
{
    visualization_msgs::Marker marker_result;
    marker_result.header.frame_id = "world";
    marker_result.type = visualization_msgs::Marker::POINTS;
    marker_result.action = visualization_msgs::Marker::MODIFY;
    marker_result.scale.x = 0.1;
    marker_result.scale.y = 0.1;
    marker_result.scale.z = 0.1;
    marker_result.pose.orientation.x = 0;
    marker_result.pose.orientation.y = 0;
    marker_result.pose.orientation.z = 0;
    marker_result.pose.orientation.w = 1;

    /* 遍历矩阵的所有元素 */
    for (int i = 0; i < esdf_matrix.rows(); i++)
    {
        for (int j = 0; j < esdf_matrix.cols(); j++)
        {
            double h = esdf_matrix(i, j);
            double max_dist = 20.0;
            if (h < -15.0 || h > 15.0) continue;
            /* 计算当前栅格中心的真实坐标 */
            double vox_pos_x, vox_pos_y;
            vox_pos_x = (j+0.5)*0.1 + (startPoint_x-0.05);
            vox_pos_y = (startPoint_y+0.05) - (i+0.5)*0.1;
            geometry_msgs::Point pt;
            pt.x = vox_pos_x;
            pt.y = vox_pos_y;
            pt.z = -0.5;
            marker_result.points.push_back(pt);

            /* 计算色彩 */
            std_msgs::ColorRGBA color;
            color.a = 1;
            std::vector<float> color_result;
            color_result = calculate_color(h, max_dist, -max_dist, R, G, B);
            color.r = color_result[0];
            color.g = color_result[1];
            color.b = color_result[2];
            marker_result.colors.push_back(color);
        }
    }
    Map_puber.publish(marker_result);
    }
/*************************************************
 * 计算渐变色
 *  输入：
 *      ESDF值；
 *      最大距离阈值；
 *      最小距离阈值；
 *      RGB色表；
 *  输出：
 *      R、G、B值；
 *************************************************/
std::vector<float>  plan_manager::calculate_color(double esdf_value, double max_dist, double min_dist, std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values)
{
    std::vector<float> color_result;

    /* 分段上色 */
    int colors_num = R_values.size();
    double dist_seg = (max_dist - min_dist) / colors_num;
    if (esdf_value > max_dist) esdf_value = max_dist;
    if (esdf_value < min_dist) esdf_value = min_dist;
    int seg_num = floor( (esdf_value - min_dist) / dist_seg );
    color_result.push_back((float)R_values[seg_num]/255.0);
    color_result.push_back((float)G_values[seg_num]/255.0);
    color_result.push_back((float)B_values[seg_num]/255.0);

    return color_result;
}

bool plan_manager::checkTrajCollision()
{   
    traj_state state;//判断算出来的轨迹是否安全
    state = SAFE;
    Eigen::Vector2i tmp_index;//
    for (size_t i = 0; i < p_.rows(); i++)
    {
        tmp_index = posToIndex(p_.row(i));
        if(esdf_map_(tmp_index(0),tmp_index(1))<=esdf_collision)
        {
            cout << "colision!"<<endl;
            state = COLLIDE;
            break;
        }
    }
    if(state == COLLIDE)
    {
        return true;//会撞
    } 
    else
        return false;//不会撞
}
Eigen::MatrixXd plan_manager::getSmoothTraj(const geometry_msgs::PoseStamped &start,
                                                                                                 const geometry_msgs::PoseStamped &end)
{
    //目前该函数仅考虑二维平面的运动，三维运动将会尽快迭代
    Eigen::MatrixXd traj;
    initial_state.resize(3,2);
    terminal_state.resize(3,2);
    initial_state <<start.pose.position.x, start.pose.position.y,
		                    1.0, 0.0,
	                        3.0, 0.0;
    terminal_state<< end.pose.position.x, end.pose.position.y,
		                    0.0, 0.0,
	                        0.0, 0.0;
    double dist = sqrt(pow(end.pose.position.x-start.pose.position.x,2)+
                                pow(end.pose.position.y-start.pose.position.y,2));
        
    opt.reset(new bspline_optimizer(Dim_,p_order_,dist));//只考虑smooth的构造
    u.reset(new UniformBspline(p_order_,opt->cps_num_,beta,Dim_,initial_state,terminal_state));
    opt->setSmoothParam(lambda1_,lambda2_,max_vel_,max_acc_);
    opt->initialControlPoints(*u);
    opt->optimizesmooth();            //计算轨迹

    u->setControlPoints(opt->control_points_);
    u->getT(TrajSampleRate);
    UniformBspline p = *u;
    UniformBspline v = p.getDerivative();
    UniformBspline a = v.getDerivative();
    p_ = p.getTrajectory(p.time_);
    v_ = v.getTrajectory(p.time_);
    a_ = a.getTrajectory(p.time_);
    traj.resize(p_.rows(),p_.cols()*3);
    //traj : px py vx vy ax ay
    for (size_t i = 0; i < traj.rows(); i++)
    {    
        traj(i,0)= p_(i,0);
        traj(i,1)= p_(i,1); 
        traj(i,2)= v_(i,0);
        traj(i,3)= v_(i,1);
        traj(i,4)= a_(i,0);
        traj(i,5)= a_(i,1);
    }   
    return traj;
}
}