#include "parseFilenameTexts.hpp"

FilenameTextParser::FilenameTextParser(string baseFolderName) {
    rgbImages = vector<string>();
    dImages = vector<string>();
    timestamps = vector<string>();
    initialRgbImage = "";
    currentIteration=0;

    string readFileName = baseFolderName+"/associations.txt";
    string line;
    ifstream datafile(readFileName);
    if (!datafile.is_open()){
        cout<<"Could not open: "<<readFileName<<" aborting process\n\n\n";
        return;
    }
    while ( getline(datafile,line) )
    {
        if (line.find('#') != string::npos){continue;}
        std::vector<std::string> splitText;
        split(splitText, line, is_any_of(" "));
        timestamps.push_back(splitText[0]);
        dImages.push_back(baseFolderName+"/"+splitText[1]);
        rgbImages.push_back(baseFolderName+"/"+splitText[3]);
    }
    datafile.close();
    
    string first_rgb_image = rgbImages[0];
    readFileName = baseFolderName+"/rgb.txt";
    datafile.open(readFileName);
    if (!datafile.is_open()){
        cout<<"Could not open: "<<readFileName<<" aborting process\n\n\n";
        return;
    }
    while ( getline(datafile,line) )
    {
        if (line.find('#') != string::npos){continue;}
        std::vector<std::string> splitText;
        split(splitText, line, is_any_of(" "));
        if (splitText[1] == first_rgb_image){break;}
        initialRgbImage = baseFolderName+"/"+splitText[1];
    }
    datafile.close();
}

FilenameTextParser::~FilenameTextParser() {
}

bool FilenameTextParser::getNextFilenames(string &rgbImageFilename, string &depthImageFilename, string &timestamp){
    return getFilenames(rgbImageFilename, depthImageFilename, timestamp, currentIteration++);
}

bool FilenameTextParser::getFilenames(string &rgbImageFilename, string &depthImageFilename, string &timestamp, int index){
    if (index>=rgbImages.size()||index>=dImages.size()||index>=timestamps.size()){return false;}

    rgbImageFilename = rgbImages[index];
    depthImageFilename = dImages[index];
    timestamp = timestamps[index];

    return true;
}

bool FilenameTextParser::setIteratorIndex(int index){
    currentIteration=index;
}