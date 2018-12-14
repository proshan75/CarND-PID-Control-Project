#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : Kp(0.0), Ki(0.0), Kd(0.0), p_error(0.0), i_error(0.0), d_error(0.0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double twiddle_tol)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Improved_p = {Kp, Ki, Kd};
    this->Tolerance = twiddle_tol;
    this->Current_index = 0;
    this->Current_iteration = 0;
    this->Error_Initialized = false;

    this->_dp = {Kp, Ki, Kd};
    this->_p = {0.0, 0.0, 0.0};
    _best_error = std::numeric_limits<double>::max();
    this->_negated = {false, false, false};
}

void PID::UpdateError(double cte)
{
    // cout << __func__ << "..." << endl;
    // if (this->Current_iteration != 0)
    // {
    //     d_error = cte - p_error;
    // }
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    // cout << "p_error: " << p_error << " i_error: " << i_error << " d_error: " << d_error << endl;
}

double PID::TotalError()
{
    // cout << __func__ << "...." << endl;
    double error = -Kp * p_error - Ki * i_error - Kd * d_error;
    // error = error - ((long) error);
    // cout << "Total error: " << error << endl;
    return error;
}

bool PID::ShouldRunTwiddle()
{
    double sum_dp = this->_dp[0] + this->_dp[1] + this->_dp[2];
    // cout << "Calculated threshold: " << sum_dp << endl;
    bool result = sum_dp > Tolerance;
    if (!result)
    {
        cout << "=================================================================" << endl;
        cout << "_dp[0]: " << _dp[0] << " _dp[1] " <<  _dp[1] << " _dp[2] " << _dp[2] << endl;
        cout << "=================================================================" << endl;
    }
    else
    {
        cout << "sum_dp " << sum_dp << endl;
        // cout << "=================================================================" << endl;
        // cout << "Kp: " << this->Kp << " Ki " << this->Ki << " Kd " << this->Kd << endl;
        // cout << "=================================================================" << endl;
    }
    return result;
}

void PID::AddError(double cte)
{
    // cout << __func__ << "...." << endl;
    this->_cte_total += (cte * cte);
    // cout << "cte_total: " << this->_cte_total << endl;
}

double PID::CalculateError(int num_of_errors)
{
    // cout << __func__ << " _cte_total: " << _cte_total << " divided by: " << num_of_errors << endl;
    double error = this->_cte_total / num_of_errors;
    cout << "Calculate Error: " << error << endl;
    return error;
}

void PID::ResetTwiddle()
{
    // cout << __func__ << "...." << endl;
    this->Error_Initialized = true;
    // this->_negated = {false, false, false};
    this->Current_iteration = 0;
    // this->Current_index = 0;
    this->_cte_total = 0.0;
    // this->_p = {0.0, 0.0, 0.0};
    // this->p_error = 0.0,
    // this->i_error = 0.0;
    // this->d_error = 0.0;
}

void PID::Twiddle(double error)
{
    cout << "... run twiddle ... index: " << this->Current_index << " iteration: " << this->Current_iteration << endl;
    //std::vector<double> p = {Kp, Ki, Kd};
    // Initialize
    // if (this->Current_iteration == 0)
    // {
        // cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
        _p[0] += _dp[0];
        _p[1] += _dp[1];
        _p[2] += _dp[2];
        cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << endl;
    // }

    // double error = this->_cte_total / (this->Current_iteration + 1);
    //cout << " error : " << error << endl;
    if (error < _best_error) //&& !_negated[Current_index]
    {
        // cout << "Improvement 1 error " << error << " compared to " << _best_error << endl;
        _best_error = error;
        // cout << " changing _dp[Current_index] from " << _dp[Current_index];
        _dp[Current_index] *= 1.1;
        // cout << " to " << _dp[Current_index] << " at index " << Current_index << endl;
        this->Improved_p[0] = _p[0];
        this->Improved_p[1] = _p[1];
        this->Improved_p[2] = _p[2];
    }
    else
    {
        // if(!_negated[Current_index])
        // {
        // cout << "Negated from " << _p[Current_index] << " to ";
        _p[Current_index] -= 2 * _dp[Current_index];
        // cout << _p[Current_index] << endl;
        _negated[Current_index] = true;
        // }
        // else
        // {
        if (error < _best_error)
        {
            // cout << "Improvement 2 error " << error << " compared to " << _best_error << endl;
            _best_error = error;
            // cout << " changing _dp[Current_index] from " << _dp[Current_index];
            _dp[Current_index] *= 1.1;
            // cout << " to " << _dp[Current_index] << " at index " << Current_index << endl;
            this->Improved_p[0] = _p[0];
            this->Improved_p[1] = _p[1];
            this->Improved_p[2] = _p[2];
        }
        else
        {
            // cout << " adding " << _dp[Current_index] << " to " << _p[Current_index];
            _p[Current_index] += _dp[Current_index];
            _dp[Current_index] *= 0.9;
            // cout << " and reducing to " << _dp[Current_index] << endl;
        }
        // }
    }

    this->Kp = _p[0];
    this->Ki = _p[1];
    this->Kd = _p[2];
    cout << ">>>>>>>>>> Kp " << this->Kp << " Ki " << this->Ki << " Kd " << this->Kd << endl;
}
