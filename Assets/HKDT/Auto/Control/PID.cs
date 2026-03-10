namespace HKDT.Auto.Control
{
    public class PID
    {
        private double pGain;
        private double iGain;
        private double dGain;
        private double integralMax;
        private double minOutput;
        private double maxOutput;

        // 過去のセンサー値を保存する
        private double prevMeasurement;
        // 過去の偏差を保存する
        private double prevProp;
        // 積算される積分値
        private double integralValue;
        // Low Path Filterしたあとの値 & 前回の微分項
        private double lowPathFiltered_D;
        // Low Path Filterの係数(0.0で遅れなし)　1に近づくほど遅れる
        private double lpfAlpha;

        private bool enable_IntegralReset;

        /// <summary>
        /// PIDを初期化する。各種設定はすべてfalseで初期化する
        /// </summary>
        /// <param name="p_gain"></param>
        /// <param name="i_gain"></param>
        /// <param name="d_gain"></param>
        /// <param name="min_output"></param>
        /// <param name="max_output"></param>
        public PID(double p_gain, double i_gain, double d_gain, double integral_max, double min_output, double max_output)
        {
            pGain = p_gain;
            iGain = i_gain;
            dGain = d_gain;

            integralMax = integral_max;

            minOutput = min_output;
            maxOutput = max_output;

            integralValue = 0.0;
            prevMeasurement = 0.0;
            prevProp = 0.0;
            lowPathFiltered_D = 0.0;

            enable_IntegralReset = false;
            lpfAlpha = 0.0;
        }

        public PID()
        {
            pGain = 0.0;
            iGain = 0.0;
            dGain = 0.0;

            integralMax = 1000.0;

            minOutput = 1000.0;
            maxOutput = 1000.0;

            integralValue = 0.0;
            prevMeasurement = 0.0;
            prevProp = 0.0;
            lowPathFiltered_D = 0.0;

            enable_IntegralReset = false;
            lpfAlpha = 0.0;
        }

        /// <summary>
        /// 符号反転時の積分項リセットを有効化できる。trueで有効化します
        /// </summary>
        /// <param name="enable"></param>
        public void IntegralReset(bool enable)
        {
            enable_IntegralReset = enable;
        }

        /// <summary>
        /// 微分項のLPFの遅れ定数を設定する0.0で遅れなし。1.0に近づくほど遅れる
        /// </summary>
        /// <param name="alpha"></param>
        public void DerivativeLPF(double alpha)
        {
            lpfAlpha = alpha;
        }

        /// <summary>
        /// PID計算を実行する
        /// </summary>
        /// <param name="target"></param>
        /// <param name="now"></param>
        /// <param name="delta_time"></param>
        /// <returns></returns>
        public double Compute(double target, double now, double delta_time)
        {
            double prop = target - now;

            if(enable_IntegralReset)
            {
                if(prevProp > 0.0 && prop < 0.0)
                {
                    integralValue = 0.0;
                }
                else if(prevProp < 0.0 && prop > 0.0)
                {
                    integralValue = 0.0;
                }
                else if(prop == 0.0)
                {
                    integralValue = 0.0;
                }
            }

            integralValue += prop * delta_time;
            if(integralValue > integralMax)
            {
                integralValue = integralMax;
            }
            else if(integralValue < -1.0 * integralMax)
            {
                integralValue = -1.0 * integralMax;
            }

            double derivative = (now - prevMeasurement) / delta_time;
            prevMeasurement = now;
            prevProp = prop;

            lowPathFiltered_D = lpfAlpha * lowPathFiltered_D + (1.0 - lpfAlpha) * derivative;

            double output = pGain * prop + iGain * integralValue + dGain * lowPathFiltered_D;

            if(output > maxOutput)
            {
                output = maxOutput;
            }
            else if(output < minOutput)
            {
                output = minOutput;
            }

            return output;
        }
    }
}