//
// Created by waxz on 19-2-20.
//

#ifndef DEMO_HMM_H
#define DEMO_HMM_H

#include <cpp_utils/serialization/matrix.h>


namespace Hmm {

    /* A Hmm model
     * 1. load and save data
     * 2. store matrix and state size : transition matrix A, observation matrix B
     * 3. fast compute state update probability: nomal update, observed update
     * */
    // save and load

    class HmmModel {
    private:
        size_t m_state_dim;
        size_t m_obs_dim;

    public:
        ublas::matrix<float> A;
        ublas::matrix<float> B;
        std::vector <ublas::matrix<float>> Bi;

        void load(std::string file_path) {

            std::ifstream ifsA(file_path + "A.txt");
            bool o = ifsA.fail();
            boost::archive::text_iarchive iarA(ifsA);

            iarA >> A;
            ifsA.close();

            std::ifstream ifsB(file_path + "B.txt");
            boost::archive::text_iarchive iarB(ifsB);

            iarB >> B;
            ifsB.close();


        }

        void save(std::string file_path) {

            std::ofstream ofsA(file_path + "A.txt");
            boost::archive::text_oarchive oarA(ofsA);
            bool o = ofsA.fail();

            oarA << A;

            std::ofstream ofsB(file_path + "B.txt");
            boost::archive::text_oarchive oarB(ofsB);

            oarB << B;

            ofsA.close();
            ofsB.close();


        }

        HmmModel(size_t state_dim, size_t obs_dim) : m_state_dim(state_dim),
                                                     m_obs_dim(obs_dim),
                                                     A(m_state_dim, m_state_dim),
                                                     B(m_state_dim, m_obs_dim) {

        }

    };

}


#endif //DEMO_HMM_H
