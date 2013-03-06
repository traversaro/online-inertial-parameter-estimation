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

#ifndef LM_MULTITASKLINEARFIXEDPARAMETERS__
#define LM_MULTITASKLINERAFIXEDPARAMETERS__

#include <string>
#include <vector>

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IParameterLearner.h"


namespace iCub {
namespace learningmachine {


/**
 * \ingroup icub_libLM_learning_machines
 *
 * Class implementing the iCub::learningmachine::IParameterLearner interfaces, 
 * but that get the parameters at the initialization and never changes them.
 * 
 * The implemented model is:
 * output = input*parameters
 * Wher output and parameters are Vectors, while input is a Matrix.
 * 
 * \see iCub::learningmachine::MultiTaskLinearGPRLearner
 *
 * \author Silvio Traversaro
 * 
 *
 */

class MultiTaskLinearFixedParameters : public IParameterLearner {
private:
    /**
     * Weight vector for the linear predictor.
     */
    yarp::sig::Vector w;
	
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
     * @param parameters fixed weight parameters to be used
     */
    MultiTaskLinearFixedParameters(unsigned int p , unsigned int m, yarp::sig::Vector parameters );
    

    /**
     * Copy constructor.
     */
    MultiTaskLinearFixedParameters(const MultiTaskLinearFixedParameters& other);

    /**
     * Destructor.
     */
    virtual ~MultiTaskLinearFixedParameters();

    /**
     * Assignment operator.
     */
    MultiTaskLinearFixedParameters& operator=(const MultiTaskLinearFixedParameters& other);

    /*
     * Inherited from IFixedSizeMatrixInputLearner.
     */
    void feedSample(const yarp::sig::Matrix& input_matrix, const yarp::sig::Vector& output);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

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
    MultiTaskLinearFixedParameters* clone() {
        return new MultiTaskLinearFixedParameters(*this);
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


    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);
    
    /*
     * Inherited from IParameterLearner
     */
    virtual yarp::sig::Vector getParameters() const;
    
    /**
     * Set desired fixed parameters, and reset
     * @param parameters desired parameters
     */
    void setParameters(const yarp::sig::Vector & parameters);
    
    /*
     * Inherited from IParameterLearner
     */
    virtual void setNoiseStandardDeviation(const yarp::sig::Vector& s);

    virtual void setWeightsStandardDeviation(const yarp::sig::Vector& s);


};

} // learningmachine
} // iCub
#endif
