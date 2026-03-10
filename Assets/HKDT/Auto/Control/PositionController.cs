namespace HKDT.Auto.Control
{
    /// <summary>
    /// 位置制御するための２階層PID
    /// 目標位置と現在位置から目標速度を算出しこの速度と現在速度から出力を計算する
    /// </summary>
    public class PositionController
    {
        PID positionPID;
        PID velocityPID;

        public PositionController()
        {
            positionPID = new PID();
            velocityPID = new PID();
        }

        /// <summary>
        /// 目標速度を計算するPIDの設定
        /// </summary>
        /// <param name="p_gain"></param>
        /// <param name="i_gain"></param>
        /// <param name="d_gain"></param>
        /// <param name="integral_max"></param>
        /// <param name="min_output"></param>
        /// <param name="max_output"></param>
        /// <param name="enable_integral_reset"></param>
        /// <param name="alpha"></param>
        public void SetPositionConfig(double p_gain, double i_gain, double d_gain, double integral_max, double min_output, double max_output, bool enable_integral_reset=false, double alpha = 0.0)
        {
            positionPID = new PID(p_gain, i_gain, d_gain, integral_max, min_output, max_output);
            positionPID.IntegralReset(enable_integral_reset);
            positionPID.DerivativeLPF(alpha);
        }
        
        /// <summary>
        /// 出力を計算するPIDの設定
        /// </summary>
        /// <param name="p_gain"></param>
        /// <param name="i_gain"></param>
        /// <param name="d_gain"></param>
        /// <param name="integral_max"></param>
        /// <param name="min_output"></param>
        /// <param name="max_output"></param>
        /// <param name="enable_integral_reset"></param>
        /// <param name="alpha"></param>
        public void SetVelocityConfig(double p_gain, double i_gain, double d_gain, double integral_max, double min_output, double max_output, bool enable_integral_reset=false, double alpha = 0.0)
        {
            velocityPID = new PID(p_gain, i_gain, d_gain, integral_max, min_output, max_output);
            velocityPID.IntegralReset(enable_integral_reset);
            velocityPID.DerivativeLPF(alpha);
        }

        /// <summary>
        /// 計算を実行する
        /// </summary>
        /// <param name="target_position"></param>
        /// <param name="now_position"></param>
        /// <param name="now_velocity"></param>
        /// <param name="delta_time"></param>
        /// <returns></returns>
        public double Compute(double target_position, double now_position, double now_velocity, double delta_time)
        {
            double target_velocity = positionPID.Compute(target_position, now_position, delta_time);
            double output = velocityPID.Compute(target_velocity, now_velocity, delta_time);

            return output;
        }
    }
}