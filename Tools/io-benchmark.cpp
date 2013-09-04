/*
    open source routing machine
    Copyright (C) Dennis Luxen, 2010

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU AFFERO General Public License as published by
the Free Software Foundation; either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
or see http://www.gnu.org/licenses/agpl.txt.
 */

#include "../Util/OSRMException.h"
#include "../Util/SimpleLogger.h"
#include "../Util/TimingUtil.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/ref.hpp>

#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <numeric>
#include <vector>

const unsigned number_of_elements = 268435456;

int main (int argc, char * argv[]) {
    LogPolicy::GetInstance().Unmute();
    try {
        SimpleLogger().Write(logDEBUG) << "starting up engines, compiled at " <<
            __DATE__ << ", " __TIME__;

        if( 1 == argc ) {
            SimpleLogger().Write(logWARNING) <<
                "usage: " << argv[0] << " /path/on/device";
            return -1;
        }

        //create file to test
        boost::filesystem::path test_path = boost::filesystem::path(argv[1]);
        test_path /= "osrm.tst";
        SimpleLogger().Write(logDEBUG) <<
            "temporary file: " << test_path.string();
        if( boost::filesystem::exists(test_path) ) {
            boost::filesystem::remove(test_path);
            SimpleLogger().Write() << "removing temporary files";
        }

        SimpleLogger().Write(logDEBUG) << "Allocating 2GB in RAM";
        std::vector<unsigned> primary_vector(number_of_elements, 0);
        std::vector<unsigned> secondary_vector(number_of_elements, 0);

        SimpleLogger().Write(logDEBUG) << "fill primary vector with data";
        std::srand ( 37337 );
        std::generate (primary_vector.begin(), primary_vector.end(), std::rand);

        std::vector<double> timing_results_1013, timing_results_random;

        SimpleLogger().Write(logDEBUG) <<
            "writing " << number_of_elements*sizeof(unsigned) << " bytes";
        //write 1GB to random filename, time everything
        boost::filesystem::ofstream test_stream(test_path, std::ios::binary);
        double time1 = get_timestamp();
        test_stream.write(
            (char *)&primary_vector[0],
            number_of_elements*sizeof(unsigned)
        );
        double time2 = get_timestamp();
        test_stream.close();
        SimpleLogger().Write(logDEBUG) <<
            "writing 1GB took " << (time2-time1)*1000 << "ms, " <<
            1024*1024/((time2-time1)*1000) << "MB/sec";

        SimpleLogger().Write(logDEBUG) <<
            "reading " << number_of_elements*sizeof(unsigned) << " bytes";
        //read and check 1GB of random data, time everything
        boost::filesystem::ifstream read_stream(test_path, std::ios::binary);
        time1 = get_timestamp();
        read_stream.read(
            (char *)&secondary_vector[0],
            number_of_elements*sizeof(unsigned)
        );
        time2 = get_timestamp();
        SimpleLogger().Write(logDEBUG) <<
            "reading 1GB took " << (time2-time1)*1000 << "ms, " <<
            1024*1024/((time2-time1)*1000) << "MB/sec";

        SimpleLogger().Write(logDEBUG) << "checking data for correctness";
        if(!std::equal(
                primary_vector.begin(),
                primary_vector.end(),
                secondary_vector.begin()
            )
        ) {
            throw OSRMException("reading data failed");
        }

        //removing any temporary data
        std::vector<unsigned>().swap(primary_vector);
        std::vector<unsigned>().swap(secondary_vector);

        //TODO: make striped reads of various sizes, log stuff to make plot
        SimpleLogger().Write(logDEBUG) << "performing 1000 gapped I/Os";
        unsigned single_element = 0;
        //read every 268435'th byte, time each I/O seperately
        for(unsigned i = 0; i < number_of_elements; i+=268435) {
            time1 = get_timestamp();
            read_stream.seekg(i*sizeof(unsigned));
            read_stream.read( (char*)&single_element, sizeof(unsigned));
            time2 = get_timestamp();
            timing_results_1013.push_back((time2-time1));
        }
        SimpleLogger().Write(logDEBUG) << "Performing gapped statistics";
        //print simple statistics: min, max, median, variance
        std::sort(timing_results_1013.begin(), timing_results_1013.end());
        double primary_sum =    std::accumulate(
                                    timing_results_1013.begin(),
                                    timing_results_1013.end(),
                                    0.0
                                );
        double primary_mean = primary_sum / timing_results_1013.size();

        double primary_sq_sum = std::inner_product(
                                    timing_results_1013.begin(),
                                    timing_results_1013.end(),
                                    timing_results_1013.begin(),
                                    0.0
                                );
        double primary_stdev = std::sqrt(
            primary_sq_sum / timing_results_1013.size() -
            (primary_mean * primary_mean)
        );

        SimpleLogger().Write() << "gapped I/O: " <<
            "min: "  << timing_results_1013.front()*1000 << ", " <<
            "max: "  << timing_results_1013.back()*1000  << ", " <<
            "mean: " << primary_mean*1000           << ", " <<
            "dev: "  << primary_stdev*1000;

        SimpleLogger().Write(logDEBUG) << "performing 1000 random I/Os";
        //make 1000 random access, time each I/O seperately
        for(unsigned i = 0; i < 1000; ++i) {
            unsigned element_to_read = std::rand()%number_of_elements;
            time1 = get_timestamp();
            read_stream.seekg(element_to_read*sizeof(unsigned));
            read_stream.read( (char*)&single_element, sizeof(unsigned));
            time2 = get_timestamp();
            timing_results_random.push_back((time2-time1));
            if(0. == timing_results_random.back()) {
                // SimpleLogger().Write() << "logged zero time at I/O " << i;
            }
        }

        SimpleLogger().Write(logDEBUG) << "Performing random statistics";
        std::sort(timing_results_random.begin(), timing_results_random.end());
        double secondary_sum =  std::accumulate(
                                    timing_results_random.begin(),
                                    timing_results_random.end(),
                                    0.0
                                );
        double secondary_mean = secondary_sum / timing_results_random.size();

        double secondary_sq_sum =   std::inner_product(
                                        timing_results_random.begin(),
                                        timing_results_random.end(),
                                        timing_results_random.begin(),
                                        0.0
                                    );
        double secondary_stdev = std::sqrt(
            secondary_sq_sum / timing_results_random.size() -
            (secondary_mean * secondary_mean)
        );

        SimpleLogger().Write() << "random I/O: " <<
            "min: "  << timing_results_random.front()*1000 << ", " <<
            "max: "  << timing_results_random.back()*1000  << ", " <<
            "mean: " << secondary_mean*1000           << ", " <<
            "dev: "  << secondary_stdev*1000;
        if( boost::filesystem::exists(test_path) ) {
            boost::filesystem::remove(test_path);
            SimpleLogger().Write(logDEBUG) << "removing temporary files";
        }
    } catch ( const std::exception & e ) {
        SimpleLogger().Write(logWARNING) << "caught exception: " << e.what();
        SimpleLogger().Write(logWARNING) << "cleaning up, and exiting";
        boost::filesystem::path test_path = boost::filesystem::path(argv[1]);
        test_path /= "osrm.tst";
        if(boost::filesystem::exists(test_path)) {
            boost::filesystem::remove(test_path);
            SimpleLogger().Write(logWARNING) << "removing temporary files";
        }
        return -1;
    }
    return 0;
}
