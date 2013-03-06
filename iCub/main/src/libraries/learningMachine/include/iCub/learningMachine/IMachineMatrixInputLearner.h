/*
 * Copyright (C) 2007-2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_IMACHINEMATRIXINPUTLEARNER__
#define LM_IMACHINEMATRIXINPUTLEARNER__

#include <string>
#include <sstream>

#include <yarp/sig/Vector.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Portable.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

#include "iCub/learningMachine/Prediction.h"
#include "iCub/learningMachine/IMachineLearner.h"

namespace iCub {
namespace learningmachine {

/**
 * \defgroup icub_libLearningMachine learningMachine
 *

 * \ingroup icub_libLM_learning_machines
 *
 * An generalized interface for a learning machine with a Matrix input. 
 * Many machine learning techniques will fall in this category.
 * Extends iCub::learningMachine::MachineLearner to support Matrix input
 * 
 * The learning machine can be used for regression and classification
 * from R^{*x*} to R^*.
 *
 *
 * \see iCub::learningMachine::MachineLearner
 * \see iCub::learningMachine::IFixedSizeMatrixInputLearner
 *
 * \author Silvio Traversaro
 *
 */

class IMachineMatrixInputLearner : public IMachineLearner {
public:
    /**
     * Constructor.
     */
    IMachineMatrixInputLearner() : IMachineLearner() { }

    /**
     * Destructor (empty).
     */
    virtual ~IMachineMatrixInputLearner() { }

    /**
     * Ask the learning machine to predict the output for a given Matrix input.
     *
     * @param input the input
     * @return the expected output
     */
    virtual Prediction predict(const yarp::sig::Matrix& input) = 0;
    
    /**
     * Provide the learning machine with an example of the desired mapping,
     * with Matrix Input.
     *
     * @param input a sample input
     * @param output the corresponding output
     */
    virtual void feedSample(const yarp::sig::Matrix& input, const yarp::sig::Vector& output) = 0;

};

} // learningmachine
} // iCub

#endif
