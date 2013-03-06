/*
 * Copyright (C) 2007-2012 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Silvio Traversaro
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

#ifndef LM_IPARAMETERLEARNER__
#define LM_IPARAMETERLEARNER__

#include <yarp/sig/Matrix.h>
#include "iCub/learningMachine/IFixedSizeMatrixInputLearner.h"


namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_learning_machines
 *
 * An generalized interface for a learning machine with a fixed domain
 * and codomain size, with a Matrix input. 
 * Many machine learning techniques will fall in this category.
 *
 * The learning machine can be used for regression and classification
 * from R^{M \times P} to R^M. M and P are here fixed sizes which should be respected
 * by the samples that are fed into the system.
 * 
 * This interface add to iCub::learningMachine::IFixedSizeMatrixInputLearner 
 * some method specific to parametric learning algorithms, useful to set or
 * obtain the learned parameters. Originally designed for inertial parameter
 * estimation, could be used for many parametric learning problems. 
 * 
 * The parametric model used is:
 * 
 * output = input * parameters 
 * 
 * Where output, parameters are vectors, and input is a matrix
 *
 * \see iCub::learningMachine::IMachineLearner
 * \see iCub::learningMachine::IFixedSizeLearner
 * \see iCub::learningMachine::IFixedSizeMatrixInputLearner
 *
 * \author Silvio Traversaro
 *
 */
class IParameterLearner : public IFixedSizeMatrixInputLearner {
    public:
        /**
         * Returns the parameters learned by the parametric learning machine.
         * The input multiplied by the learned parameters returns the 
         * predicted output.
         * 
         * @return a yarp::sig::Vector of the learned parameters
         */
        virtual yarp::sig::Vector getParameters() const = 0;
        
        /**
         * Set the standard deviation of the output noise. 
         * The covariance matrix of the output noise is set to a diagonal
         * matrix with the diagonal elements corresponding to the square 
         * (variance) of the submitted standard deviations. 
         * 
         * @param s a yarp::sig::Vector 
         */
        virtual void setNoiseStandardDeviation(const yarp::sig::Vector& s) = 0;
        
        /**
         * Set the standard deviation of the prior distribution of the weights. 
         * The covariance matrix of the prior distribution of the weights is set to a diagonal
         * matrix with the diagonal elements corresponding to the square 
         * (variance) of the submitted standard deviations. 
         * 
         * @param s a yarp::sig::Vector 
         */
        virtual void setWeightsStandardDeviation(const yarp::sig::Vector& s) = 0;
        
        

};

} // learningmachine
} // iCub

#endif

