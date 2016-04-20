#include <ros/ros.h>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>
#include <boost/filesystem.hpp>
   
#include <sstream>
 
bool isDirectory(const std::string& path)
{   
    boost::filesystem::path dir(path);
    return boost::filesystem::is_directory(dir) 
        || (!path.empty() && dir.extension().empty()); // or non-existing directory
}


std::string swapFileExtension(const std::string& in, const std::string& ext, bool returnOnlyFilename)
{
    boost::filesystem::path outPath(in);
    boost::filesystem::path swapped = outPath.replace_extension(ext);
    if (returnOnlyFilename) return swapped.filename().string();
    return swapped.string();
}

std::string fileExtension(const char* file)
{
    boost::filesystem::path dPath(file);
    return dPath.extension().string();
}


bool makeDirectoryIfNeeded(const char * dPath)
{   
    try
    {
        boost::filesystem::path dir(dPath);
        boost::filesystem::path buildPath;
    
        for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end()); it != it_end; ++it)
        {
            buildPath /= *it;
            //std::cout << buildPath << std::endl;
    
            if (!boost::filesystem::exists(buildPath) &&
                    !boost::filesystem::create_directory(buildPath))
            {
                ROS_ERROR_STREAM("Could not create directory " << buildPath);
                return false;
            }
        }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        return false;
    }
    return true;
}


/**
 * \param outFormat output format string
 */
bool convert(const std::string& inFile, const std::string& outFile, const std::string& outFormat)
{
    // Create an instance of the Importer class
    Assimp::Importer importer;
    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll 
    // propably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( inFile, 
                aiProcess_Triangulate);
                /*aiProcess_OptimizeMeshes | 
                /aiProcess_CalcTangentSpace             | 
                aiProcess_Triangulate                        |
                aiProcess_JoinIdenticalVertices    |
                aiProcess_SortByPType);*/
    
    // If the import failed, report it
    if( !scene)
    {
        ROS_ERROR_STREAM("Could not import file "<<inFile);
        return false;
    }
   
    Assimp::Exporter exporter;
    if (exporter.Export(scene,outFormat,outFile) != AI_SUCCESS)
    {
        ROS_ERROR_STREAM("Could not export "<<inFile<<" to "<<outFormat);
        return false;
    }     
    return true;
}




bool convertFiles(const std::string& input, const std::string& input_format,
    const std::string& output, const std::string& output_format, bool recursive)
{
    ROS_INFO_STREAM("Converting all files with extension "<<input_format
        <<" in "<<input<<" and writing them to "<<output<<" as "<<output_format);
    boost::filesystem::path p(input);
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if (boost::filesystem::is_regular_file(itr->path()))
        {
            std::string current_file = itr->path().string();
            std::string extension = itr->path().extension().string();
            extension.erase(0,1); //remove the dot
            // ROS_INFO_STREAM("Current file: "<<current_file<<" with extension "<<extension);

            if (extension == input_format)
            {
                // ROS_INFO_STREAM("Found a file of type "<<input_format<<": "<<current_file);
                std::string swappedFile = swapFileExtension(current_file, output_format, true);
                std::stringstream str;
                str<<output<<"/"<<swappedFile;
                std::string out_file = str.str();
                ROS_INFO_STREAM("Processing "<<current_file<<", new filename "<<swappedFile<<", saving as "<<out_file);
                if (!convert(current_file, out_file, output_format))
                {
                    ROS_ERROR("Could not convert");
                    return false;
                }
            }
        }
        else if (recursive)
        {
            std::string next_input_dir = itr->path().string();
            std::string next_output_dir =  output + "/" + itr->path().filename().string();
            ROS_INFO_STREAM("New input: "<<next_input_dir<<", output="<<next_output_dir);
            if (!convertFiles(next_input_dir,input_format, next_output_dir, output_format, recursive))
            {
                ROS_ERROR_STREAM("Could not convert in recursion directory "<<next_input_dir);
            }
        }
    }
    return true;
}




void usage(char * prog)
{
    ROS_INFO_STREAM("Usage: (arguments in fixed order!): " << prog <<
                    " <in-file-or-dir> <out-format> <out-file-or-dir> [<in-format>]");
    ROS_INFO("If <in-file-or-dir> is a directory, then all files of the <in-format> are processed (<out-file-or-dir> has to be a directory too).");
    ROS_INFO("<out-file-or-dir> is a directory, the original filename will be used and the result saved in the directory.");
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "assimp_collada_converter", ros::init_options::AnonymousName);
    ros::NodeHandle priv("~");
    ros::NodeHandle pub("");

    if (argc < 4)
    {
        ROS_ERROR("Not enough arguments!");
        usage(argv[0]);
        return 0;
    }

    // set parameters

    std::string input = std::string(argv[1]);
    ROS_INFO("Input: %s", input.c_str());
    
    std::string output_format = std::string(argv[2]);
    ROS_INFO("Output format: %s", output_format.c_str());
    
    std::string output = std::string(argv[3]);
    ROS_INFO("Output: %s", output.c_str());

    std::string input_format;
    if (argc > 4)
    {
        input_format = std::string(argv[4]);
        ROS_INFO("Processing files of format: %s", input_format.c_str());
    }

    bool inputIsDir = isDirectory(input); 
    bool outputIsDir = isDirectory(output);
   
    if (inputIsDir && !outputIsDir)
    {
        ROS_ERROR("If input is directory, output has to be as well");
        usage(argv[0]);
        return 0;
    }

    if (outputIsDir)
    {
        makeDirectoryIfNeeded(output.c_str());
    }

    if (!inputIsDir)
    {   // simple case: just convert one file
        std::string out_file=output;
        if (outputIsDir)
        {
            std::string swappedFile = swapFileExtension(input, output_format, true);
            std::stringstream str;
            str<<output<<"/"<<swappedFile;
            out_file = str.str();
            ROS_INFO_STREAM("Swapped extension of "<<input<<" to "<<swappedFile<<", saving as "<<out_file);
        }
        if (!convert(input, out_file, output_format))
        {
            ROS_ERROR("Could not convert");
        }
        else
        {
            ROS_INFO_STREAM("Conversion successful. Output written to "<<output);
        }
        return 0;
    }
    
    // input and output are directories
    

    if (input_format.empty())
    {
        ROS_ERROR("Have to specify input format if processing all files of a directory");
        usage(argv[0]);
        return 0;
    }


    bool recursive=true;
    if (!convertFiles(input,input_format, output, output_format, recursive))
    {
        ROS_ERROR_STREAM("Could not convert.");
    }

    
    /*ROS_INFO_STREAM("Converting all files with extension "<<input_format
        <<" in "<<input<<" and writing them to "<<output<<" as "<<output_format);

    boost::filesystem::path p(input);
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if (boost::filesystem::is_regular_file(itr->path()))
        {
            std::string current_file = itr->path().string();
            std::string extension = itr->path().extension().string();
            extension.erase(0,1); //remove the dot
            // ROS_INFO_STREAM("Current file: "<<current_file<<" with extension "<<extension);

            if (extension == input_format)
            {
                // ROS_INFO_STREAM("Found a file of type "<<input_format<<": "<<current_file);
                std::string swappedFile = swapFileExtension(current_file, output_format, true);
                std::stringstream str;
                str<<output<<"/"<<swappedFile;
                std::string out_file = str.str();
                ROS_INFO_STREAM("Processing "<<current_file<<", new filename "<<swappedFile<<", saving as "<<out_file);
                if (!convert(current_file, out_file, output_format))
                {
                    ROS_ERROR("Could not convert");
                }
            }
        }
    }*/

    ROS_INFO_STREAM("Conversion successful. Output written to "<<output);
    return 0;
}
