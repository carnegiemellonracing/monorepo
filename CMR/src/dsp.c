/**
 * @file dsp.c
 * @brief Digital Signal Processing functions
 *
 * @author Carnegie Mellon Racing
 */

#ifdef L431

#include <stdlib.h>
#include <CMR/dsp.h>

/** @brief Filter coefficients for each desired corner.
 * We could add the matlab code to generate these in the documentation,
 * but probably wouldn't be of significant use.
 */
static const float32_t cmr_dspFilterCoeffs[CMR_DSP_FILTER_NUM][5*CMR_DSP_FILTER_NUM_STAGES] = {
    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 0.4Hz (-3dB point)
     */
    [CMR_DSP_FILTER_1HZ] = {
        1.5783620E-06, 3.1567240E-06, 1.5783620E-06, 1.9990135E+00, -9.9901985E-01,
        1.5769340E-06, 3.1538680E-06, 1.5769340E-06, 1.9972050E+00, -9.9721130E-01,
        1.5758428E-06, 3.1516856E-06, 1.5758428E-06, 1.9958230E+00, -9.9582930E-01,
        1.5752529E-06, 3.1505058E-06, 1.5752529E-06, 1.9950759E+00, -9.9508216E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 2Hz (-3dB point)
     */
    [CMR_DSP_FILTER_5HZ] = {
        3.9381354E-05, 7.8762708E-05, 3.9381354E-05, 1.9949514E+00, -9.9510896E-01,
        3.9204201E-05, 7.8408402E-05, 3.9204201E-05, 1.9859773E+00, -9.8613417E-01,
        3.9069687E-05, 7.8139374E-05, 3.9069687E-05, 1.9791632E+00, -9.7931951E-01,
        3.8997273E-05, 7.7994546E-05, 3.8997273E-05, 1.9754949E+00, -9.7565092E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 4Hz (-3dB point)
     */
    [CMR_DSP_FILTER_10HZ] = {
        1.5713498E-04, 3.1426996E-04, 1.5713498E-04, 1.9896140E+00, -9.9024256E-01,
        1.5573111E-04, 3.1146223E-04, 1.5573111E-04, 1.9718385E+00, -9.7246141E-01,
        1.5467347E-04, 3.0934694E-04, 1.5467347E-04, 1.9584468E+00, -9.5906550E-01,
        1.5410705E-04, 3.0821409E-04, 1.5410705E-04, 1.9512749E+00, -9.5189129E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 10Hz (-3dB point)
     */
    [CMR_DSP_FILTER_25HZ] = {
        9.7469593E-04, 1.9493919E-03, 9.7469593E-04, 1.9718981E+00, -9.7579684E-01,
        9.5337764E-04, 1.9067553E-03, 9.5337764E-04, 1.9287692E+00, -9.3258273E-01,
        9.3768096E-04, 1.8753619E-03, 9.3768096E-04, 1.8970134E+00, -9.0076413E-01,
        9.2939962E-04, 1.8587992E-03, 9.2939962E-04, 1.8802595E+00, -8.8397712E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 40Hz (-3dB point)
     */
    [CMR_DSP_FILTER_100HZ] = {
        1.4981559E-02, 2.9963119E-02, 1.4981559E-02, 1.8475297E+00, -9.0745598E-01,
        1.3801535E-02, 2.7603069E-02, 1.3801535E-02, 1.7020088E+00, -7.5721493E-01,
        1.3016825E-02, 2.6033650E-02, 1.3016825E-02, 1.6052382E+00, -6.5730554E-01,
        1.2628246E-02, 2.5256493E-02, 1.2628246E-02, 1.5573186E+00, -6.0783158E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 100Hz (-3dB point)
     */
    [CMR_DSP_FILTER_250HZ] = {
        8.5667865E-02, 1.7133573E-01, 8.5667865E-02, 1.4515796E+00, -7.9425105E-01,
        7.1984525E-02, 1.4396905E-01, 7.1984525E-02, 1.2197254E+00, -5.0766347E-01,
        6.4143120E-02, 1.2828624E-01, 6.4143120E-02, 1.0868585E+00, -3.4343094E-01,
        6.0572179E-02, 1.2114436E-01, 6.0572179E-02, 1.0263515E+00, -2.6864019E-01,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 200Hz (-3dB point)
     */
    [CMR_DSP_FILTER_500HZ] = {
        2.9142074E-01, 5.8284148E-01, 2.9142074E-01, 5.2130927E-01, -6.8699222E-01,
        2.2605098E-01, 4.5210196E-01, 2.2605098E-01, 4.0437229E-01, -3.0857621E-01,
        1.9292853E-01, 3.8585705E-01, 1.9292853E-01, 3.4512104E-01, -1.1683514E-01,
        1.7875346E-01, 3.5750691E-01, 1.7875346E-01, 3.1976390E-01, -3.4777725E-02,
    },

    /* 8th Order Butterworth IIR Low-Pass Filter
     * Sampling Rate: 1000Hz
     * Cutoff Frequency: 400Hz (-3dB point)
     */
    [CMR_DSP_FILTER_1000HZ] = {
        8.1145766E-01, 1.6229153E+00, 8.1145766E-01, -1.4515796E+00, -7.9425105E-01,
        6.8184721E-01, 1.3636944E+00, 6.8184721E-01, -1.2197254E+00, -5.0766347E-01,
        6.0757235E-01, 1.2151447E+00, 6.0757235E-01, -1.0868585E+00, -3.4343094E-01,
        5.7374792E-01, 1.1474958E+00, 5.7374792E-01, -1.0263515E+00, -2.6864019E-01,
    },

};

/**
 * @brief Initialize a preset filter
 *
 * @param filter The object to init
 * @param sel The corner of the preset filter
 */
void cmr_dspFilterInit(cmr_dspFilter_t *filter, cmr_dspFilterSelection_t sel) {
    filter->_instance.numStages = CMR_DSP_FILTER_NUM_STAGES;
    filter->_instance.pState = filter->_buf;
    filter->_instance.pCoeffs = (float32_t*) &cmr_dspFilterCoeffs[sel];

    memset(filter->_buf, 0, sizeof(filter->_buf));
}

/**
 * @brief Consume/process a sample via the iterative filter
 *
 * @note Try to call this function with a real frequency
 * as close to the assumed sampling frequency (1KHz for all corners).
 *
 * @param filter The filter in use
 * @param in Most recent sample
 * @param out The current (rolling) filter output
 */
void cmr_dspFilterProcess(cmr_dspFilter_t *filter, float32_t *in, float32_t *out){
    arm_biquad_cascade_df2T_f32(&filter->_instance, in, out, 1);
}

/**
 * @brief Same as cmr_dspFilterProcess, but for integers/fixed point
 *
 * @param filter The filter in use
 * @param in Most recent sample
 * @return uint32_t The current (rolling) filter output
 */
uint32_t cmr_dspFilterProcessFixed(cmr_dspFilter_t *filter, uint32_t in){
    float32_t in_f = (float32_t) in;
    float32_t out_f;
    cmr_dspFilterProcess(filter, &in_f, &out_f);
    return (uint32_t) out_f;
}

#endif /* L431 */