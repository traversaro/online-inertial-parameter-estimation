/*
 * author:  Silvio Traversaro
 * email:   pegua1@gmail.com
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef LM_MULTITASKLINEARGPRLEARNERFIXEDPARAMETERS__
#define LM_MULTITASKLINEARGPRLEARNERFIXEDPARAMETERS__

#include <string>
#include <vector>

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IParameterLearner.h"


namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_learning_machines
 *
 * Standard linear Bayesian regression or, equivalently, Gaussian Process
 * Regression with a linear covariance function, with multiple output. 
 * It uses a repeated rank 1 update rule to
 * incrementally update the Cholesky factor of the covariance matrix.
 * 
 * This class try to fit the data in this model:
 * \f[
 *   y = X^{\top} w + \epsilon_n 
 * \f]
 * 
 * Where \f X^{\top} \in \mathbb{R}^{m \times p} \f is the input matrix
 * and \f y \in \mathbb{R}^{m} \f is the output vector
 * \f \espilon_n \in \mathbb{R}^{m} \f is the output error, subject to the prior \f \mathcal{N}(0,\Sigma_n) \f.
 * \f w \in \mathbb{R}^{m} \f the weights vector, with prior \f \mathcal{N}(\hat{w},\Sigma_w) \f.
 * It is possible to define the priors (with covariance restricted to be in diagonal form), 
 * the default values are \f \hat{w} = 0 \f, \f (\Sigma_w)_{i,i} \to \inf \f (no information on the weights) 
 * \f (\Sigma_n)_{j,j} \to 0 \f (no output error). The default values correspond to a standard least square regression.
 * with no regularization.
 * 
 *
 * See:
 * Learning Inverse Dynamics by Gaussian process Begrression under the Multi-Task Learning Framework
 * Yeung, D.Y. and Zhang, Y.
 * The Path to Autonomous Robots
 * page 135, formula 8.5
 *
 * Gaussian Processes for Machine Learning.
 * Carl Edward Rasmussen and Christopher K. I. Williams.
 * The MIT Press, 2005.
 *
 * \see iCub::learningmachine::IMachineLearner
 * \see iCub::learningmachine::IFixedSizeMatrixInputSizeLearner
 * \see iCub::learningmachine::RLSLearner
 * \see iCub::learningmachine::LinearGPRLearner
 *
 * \author Arjan Gijsberts
 * \author Silvio Traversaro
 * 
 * This class try to fit the data in this model:
 * 
 *
 */

class MultiTaskLinearGPRLearnerFixedParameters : public IParameterLearner {
private:
    /**
     * Cholesky factor of the covariance matrix.
     */
    yarp::sig::Matrix R;

    /**
     * Vector b 
     */
    yarp::sig::Vector b;

    /**
     * Weight vector for the linear predictor.
     */
    yarp::sig::Vector w;
    
    /**
     * Inverse of the covariance matrix (A), updated only until it is full rank,
     * then the Cholesky factor R of the inverse is calculated
     */
    yarp::sig::Matrix A;
    
    /**
     * First part of weight vector w, fixed and not learned
     */
     yarp::sig::Vector fixed_w;
    
    /*********
     * PRIOR INFORMATION
     ********/
          
    /**
     * Inverse of the prior covariance matrix for the output noise
     */
     yarp::sig::Matrix inv_Sigma_n;
     
    /**
     * Inverse of the prior covariance matrix for the output noise
     */
     yarp::sig::Matrix inv_Sigma_w;
    
    /**
     * Flag, true if no output error is considered 
     */
    bool no_output_error;
    
    /**
     * Flag, true if the prior on the weight is not considered
     */
    bool weight_prior_indefinite;
    
    /**
     * Flag, true if A is still not full rank
     */
    bool A_not_full_rank;
	
    /**
     * Number of samples during last training routine
     */
    int sampleCount;

public:
    /**
     * Constructor.
     *
     * @param p initial domain size (number of features, p)
     * @param m initial codomain size (number of outputs, m)
     */
    MultiTaskLinearGPRLearnerFixedParameters(unsigned int p, unsigned int m, const yarp::sig::Vector & fixed_parameters);
    

    /**
     * Copy constructor.
     */
    MultiTaskLinearGPRLearnerFixedParameters(const MultiTaskLinearGPRLearnerFixedParameters& other);

    /**
     * Destructor.
     */
    virtual ~MultiTaskLinearGPRLearnerFixedParameters();

    /**
     * Assignment operator.
     */
    MultiTaskLinearGPRLearnerFixedParameters& operator=(const MultiTaskLinearGPRLearnerFixedParameters& other);

    /*
     * Inherited from IFixedSizeMatrixInputLearner.
     */
    void feedSample(const yarp::sig::Matrix& input_matrix, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();
    
    /*
     * Inherited from IMachinerLearner
     */    
    virtual Prediction predict(const yarp::sig::Vector& input);

    /*
     * Inherited from IMachineMatrixInputLearner.
     */
    virtual Prediction predict(const yarp::sig::Matrix& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    MultiTaskLinearGPRLearnerFixedParameters* clone() {
        return new MultiTaskLinearGPRLearnerFixedParameters(*this);
    }
    
    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(yarp::os::Bottle& bot);
    
    
    /**
     * Sets the signal noise standard deviation to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setNoiseStandardDeviation(double s);

    /**
     * Sets the signal noise standard deviation to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setNoiseStandardDeviation(const yarp::sig::Vector& s);

    /**
     * Accessor for the signal noise standard deviation.
     *
     * @returns the value of the parameter
     */
    yarp::sig::Vector getNoiseStandardDeviation();
    
    /**
     * Sets the weights prior standard deviation to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setWeightsStandardDeviation(double s);

    /**
     * Sets the weights prior standard deviation to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setWeightsStandardDeviation(const yarp::sig::Vector& s);

    /**
     * Accessor for the weights standard deviation.
     *
     * @returns the value of the parameter
     */
    yarp::sig::Vector getWeightsStandardDeviation();


    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);
    
    /**
     * Inherited from IParameterLearner
     */
    virtual yarp::sig::Vector getParameters() const;
    



};

} // learningmachine
} // iCub
#endif
