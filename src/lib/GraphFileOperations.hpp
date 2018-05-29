/**
  LibFloorplan
  Author: Alper Aydemir. Jan, 2012.
*/

#include "FloorPlanGraph.hpp"
#include "GraphUtils.hpp"

// Boost includes
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/iteration_macros.hpp>

// OpenCV inclues
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// std includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

// linux includes
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef GRAPHFILEOPERATIONS_H
#define GRAPHFILEOPERATIONS_H


using namespace boost;
using namespace boost::property_tree;
using namespace std;

namespace floorplan {

class GraphFileOperations {
public:
 static   string GetAttr(const ptree::value_type v, std::string attrName)
    {
        using boost::property_tree::ptree;
        string ret = "";

        BOOST_FOREACH(const ptree::value_type &i,
                      v.second.get_child("<xmlattr>"))
        {
            if( ((string)i.first.data()).compare(attrName) == 0 )
                ret = i.second.data();
        }
        return ret;
    }

   static string formatStringForDot(string str){
        if (str.size() != 0){
            if (isdigit(str[0]))
                str = "S" + str;

            string illegalChars = "-* ";
            size_t found=str.find_first_of(illegalChars);
            while (found!=string::npos)
            {
                str[found]='_';
                found=str.find_first_of(illegalChars,found+1);
            }
        }
        return str;

    }

   static double str2double(char* str)
    {
        char* endptr;
        double value = strtod(str, &endptr);
        if (*endptr) return 0;
        return value;
    }

    /**
  * Load a graph from XML file. Both MIT and KTH formats are supported.
  */
   static graphProperties loadGraphFromXML(std::string filenamePath, floorplanGraph& outGraph, std::string rootNodeName){
        ptree XMLtree;
        read_xml(filenamePath, XMLtree);
        map<string, Vertex> nameToVertex;
        graphProperties currentGraphProperty;
        currentGraphProperty.filepath = filenamePath;

        size_t foundDot = filenamePath.find_last_of('/');
        if (foundDot != string::npos)
            currentGraphProperty.floorname = filenamePath.substr(foundDot+1, filenamePath.size()-1);
        else
            currentGraphProperty.floorname = filenamePath;

        BOOST_FOREACH(ptree::value_type &v, XMLtree.get_child(rootNodeName.c_str()))
        {
            if( ((string)v.first.data()).compare("space") == 0 )
            {
                string spaceName = GetAttr(v, "name");
                string spaceType = GetAttr(v, "type");

                size_t foundDot = spaceName.find_first_of('.');
                if (foundDot != string::npos)
                    spaceName = spaceName.substr(0, foundDot);


                if (!spaceName.empty()){
                    pair<bool,Vertex> searchResult =  GraphUtils::doesVertexExists(spaceName, outGraph);
                    Vertex currentVertex;
                    if (!searchResult.first){
                        currentVertex = add_vertex(outGraph);
                        outGraph[currentVertex].category  =  spaceType ;
                        outGraph[currentVertex].vertex_id =  spaceName;
                        nameToVertex[spaceName] = currentVertex;
                    }
                    else {

                        currentVertex = searchResult.second;
                    }

                    // For each space's children
                    BOOST_FOREACH(ptree::value_type &i,
                                  v.second)
                    {
                        if (((string)i.first.data()).compare("contour") == 0){
                            vector<Point2D> points;
                            BOOST_FOREACH(ptree::value_type &l,
                                          i.second)
                            {

                                char* endptr;
                                if( ((string)l.first.data()).compare("centroid") == 0 ){
                                    outGraph[currentVertex].centroid.x = strtod(GetAttr(l, "x").c_str(), &endptr);
                                    outGraph[currentVertex].centroid.y = strtod(GetAttr(l, "y").c_str(), &endptr);
                                }
                                else if( ((string)l.first.data()).compare("extent") == 0 ){

                                    outGraph[currentVertex].maxx = strtod(GetAttr(l, "maxx").c_str(), &endptr);
                                    outGraph[currentVertex].maxy = strtod(GetAttr(l, "maxy").c_str(), &endptr);
                                    outGraph[currentVertex].minx = strtod(GetAttr(l, "minx").c_str(), &endptr);
                                    outGraph[currentVertex].miny = strtod(GetAttr(l, "miny").c_str(), &endptr);
                                }
                                else if( ((string)l.first.data()).compare("point") == 0 ){
                                    points.push_back(Point2D(strtod(GetAttr(l, "x").c_str(), &endptr), strtod(GetAttr(l, "y").c_str(), &endptr)));
                                }
                                else if ( ((string)l.first.data()).compare("linesegment") == 0 ) {
                                    LineSegment lsegment;
                                    lsegment.startPos = Point2D(
                                        strtod(GetAttr(l, "x1").c_str(), &endptr),
                                        strtod(GetAttr(l, "y1").c_str(), &endptr)
                                    );

                                    lsegment.endPos = Point2D(
                                        strtod(GetAttr(l, "x2").c_str(), &endptr),
                                        strtod(GetAttr(l, "y2").c_str(), &endptr)
                                    );

                                    lsegment.type = GetAttr(l, "type");
                                    lsegment.portalToRoom = GetAttr(l, "target");
                                    outGraph[currentVertex].roomLayout.push_back(lsegment);
                                }
                            } // for each contour children

                            if(points.size() > 0){
                                LineSegment lsegment;
                                for (int i=0; i < points.size(); i++){
                                    if(i == points.size()-1){
                                        lsegment.startPos = points[i];
                                        lsegment.endPos = points[0];
                                    }
                                    else{
                                        lsegment.startPos = points[i];
                                        lsegment.endPos = points[i+1];
                                    }
                                    outGraph[currentVertex].roomLayout.push_back(lsegment);
                                }
                            }
                            outGraph[currentVertex].updateExtent();

                        } // if it's a contour

                    } // for each space children
                } // if space with an non-empty name
            } // if space
            else if ( ((string)v.first.data()).compare("Scale") == 0 )
            {
                char* endptr;
                currentGraphProperty.pixel_distance = strtod(GetAttr(v, "PixelDistance").c_str(), &endptr);
                currentGraphProperty.real_distance = strtod(GetAttr(v, "RealDistance").c_str(), &endptr);
            } // if Scale
        }


        // now add edges

        BOOST_FOREACH(ptree::value_type &v, XMLtree.get_child(rootNodeName.c_str()))
        {
            if( ((string)v.first.data()).compare("space") == 0 )
            {
                string sourceSpaceName = GetAttr(v, "name");
                size_t foundDot = sourceSpaceName.find_first_of('.');
                if (foundDot != string::npos)
                    sourceSpaceName = sourceSpaceName.substr(0, foundDot);
                BOOST_FOREACH(ptree::value_type &i,
                              v.second)
                {
                    if( ((string)i.first.data()).compare("portal") == 0 )
                    {

                        BOOST_FOREACH(ptree::value_type &j,
                                      i.second.get_child("<xmlattr>"))
                        {
                            if( ((string)j.first.data()).compare("target") == 0 )
                            {
                                string targetSpaceName = (string)j.second.data();
                                size_t foundDot = targetSpaceName.find_first_of('.');
                                if (foundDot != string::npos)
                                    targetSpaceName = targetSpaceName.substr(0, foundDot);

                                if (nameToVertex.count(targetSpaceName) != 0 && nameToVertex.count(sourceSpaceName) !=  0) {
                                    Vertex v1 = nameToVertex[sourceSpaceName];
                                    Vertex v2 = nameToVertex[targetSpaceName];
                                    pair<Edge, bool> ret = boost::edge(v1, v2, outGraph);
                                    if (!ret.second && targetSpaceName.compare(sourceSpaceName) != 0){
                                        add_edge(v1, v2, outGraph);
                                    } // if edge does not already exists and target,source are not the same
                                } // if both target and source exists in the graph
                            } // if it's a target attribute
                        } // for each portal's attributes
                    } // if this is a portal

                } // for each element of space
            } // if this is a space
        } // Adding edges end

        *(outGraph.m_property) = currentGraphProperty;
        return currentGraphProperty;
    }




  /**
  * Load all graphs in a folder
  */
 static   std::vector<graphProperties> loadAllGraphsInFolder(string sDir, vector<floorplanGraph>& vResult, std::string rootNodeName, int iLimit = -1)
    {
        DIR *dp;
        stringstream fLoad;
        struct dirent *dirp;

        vector<graphProperties> retGraphProperties;
        if((dp  = opendir(sDir.c_str())) == NULL)
        {
            cout << "Error(" << errno << ") opening " << sDir << endl;
            return retGraphProperties;
        }

        int iCounter = 0;

        //int iFiles = CountFilesInDir(sDir);
        //vResult.reserve(iFiles);

        // Load all the files (except current directory(.) and previous dir(..))
        while ((dirp = readdir(dp)) != NULL )
        {
           // cout << dirp->d_name << endl;
            string str(dirp->d_name);
            if( strlen(dirp->d_name) != 0 && dirp->d_name[0] != '.')
            {
                fLoad << sDir << "/" << dirp->d_name;

                struct stat statTmp;
                stat(fLoad.str().c_str(), &statTmp);

                // Check if it is a file or directory
                if( !S_ISDIR(statTmp.st_mode) && fLoad.str().find(".xml") != string::npos)
                {
                    cout << "Loading file " << fLoad.str() << endl;
                    floorplanGraph Graph;
                    try {
                      graphProperties props = loadGraphFromXML(string(fLoad.str()), Graph, rootNodeName);
                      retGraphProperties.push_back(props);
                      vResult.push_back(Graph);
                      fLoad.str("");
                     }
                    catch( const boost::exception& E) {
                      cerr << "Could not load " << fLoad.str() << endl; 
                    }
               }
                else
                {
                    if(S_ISDIR(statTmp.st_mode)){
                        cout << "Loading directory " << fLoad.str() << endl;
                        vector<graphProperties> tmp = loadAllGraphsInFolder(string(fLoad.str()), vResult, rootNodeName, iLimit);
                        retGraphProperties.insert(retGraphProperties.end(),tmp.begin(),tmp.end());
                    }
                    fLoad.str("");
                }
            }
            if( iLimit != -1 && iCounter >= iLimit )
                return retGraphProperties;
        }


        closedir(dp);

        return retGraphProperties;
    }


    /**
  * Write a graph to file with XML format
  */
   static void saveGraphToXML(std::string filenamePath, const floorplanGraph& Graph){

    }

    /**
  * Write a graph to file with Matlab-friendly format
  */
    static void saveGraphToMatlab(std::string filenamePath, const floorplanGraph& Graph) {

        std::ofstream out((filenamePath).c_str());

        BGL_FORALL_EDGES(e, Graph, floorplanGraph){
            string sourcestr = formatStringForDot(Graph[source(e, Graph)].vertex_id);
            int sourceid; //= source(e, Graph);
            string targetstr = formatStringForDot(Graph[target(e, Graph)].vertex_id);
            int targetid; // = target(e, Graph);
            out << sourceid << " " <<  targetid << std::endl;
        }
        out.close();
    }


    /**
  * Write a graph to file with Dot format
  */
  static  void saveGraphToDot(std::string filenamePath, const floorplanGraph& Graph){

        std::ofstream out((filenamePath).c_str());
        out << "graph TopoGraph {" << endl;
        out << "overlap=false; splines=true;" << endl;
        BGL_FORALL_VERTICES(v, Graph, floorplanGraph){
            string name = formatStringForDot(Graph[v].vertex_id);
            string category =formatStringForDot(Graph[v].category);
            out << name << " [shape=ellipse" << " label=\"" << name << " - " << Graph[v].category << "\"];" << std::endl;
        }


        BGL_FORALL_EDGES(e, Graph, floorplanGraph){
            string sourcestr = formatStringForDot(Graph[source(e, Graph)].vertex_id);
            string targetstr = formatStringForDot(Graph[target(e, Graph)].vertex_id);
            out << sourcestr << " -- " <<  targetstr << ";" << std::endl;
        }

        out <<"}";
        out.close();

    }

  /**
* Write a graph to file with Dot format
*/
static  void saveGraphToDot(std::string filenamePath, const simpleGraph& Graph){

      std::ofstream out((filenamePath).c_str());
      out << "graph TopoGraph {" << endl;
      out << "overlap=false; splines=true;" << endl;
      BGL_FORALL_VERTICES(v, Graph, simpleGraph){
          out << v << " [shape=ellipse" << " label=\"" << v << "\"];" << std::endl;
      }


      BGL_FORALL_EDGES(e, Graph, simpleGraph){
          simpleVertex sourcestr = source(e, Graph);
          simpleVertex targetstr = target(e, Graph);
          out << sourcestr << " -- " <<  targetstr << ";" << std::endl;
      }

      out <<"}";
      out.close();
  }

/**
* Visualize a graph with the neato tool and write to file.
*/
static   void saveGraphToPNG(std::string filenamePath, const simpleGraph& Graph){
    saveGraphToDot(filenamePath,Graph);
    system(("neato -Tpng -Gcharset=latin1 " + filenamePath + " > " + filenamePath + ".png").c_str());
}


    /**
  * Visualize a graph with the neato tool and write to file.
  */
 static   void saveGraphToPNG(std::string filenamePath, const floorplanGraph& Graph){
        saveGraphToDot(filenamePath,Graph);
        system(("neato -Tpng -Gcharset=latin1 " + filenamePath + " > " + filenamePath + ".png").c_str());
    }
  
  /**
   * @brief transforms from graph coordinates to map coordinate of the given size and resolution
   * @param graph the floorplan graph
   * @param point the point in graph coordinates whose map coordinates are required
   * @param map_size the size of the output map
   * @param target_resolution target resolution of the output map (m/pixels)
   * @return point in map frame
   */
  static Point2D transformToMapCoords(const floorplanGraph &graph, Point2D point, cv::Size map_size, double target_resolution)
  {
    Point2D map_centroid(map_size.width/2.0, map_size.height/2.0);
    
    // m/pixel in the current scale
    double current_resolution = graph.m_property->real_distance / graph.m_property->pixel_distance;
    auto offsetted_point = point - graph.m_property->centroid;
    auto map_point = offsetted_point * current_resolution / target_resolution + map_centroid;

    return map_point;
  }

  /**
   * @brief transforms from mpa coordinates to graph coordinate of the given map size and resolution
   * @param graph the floorplan graph
   * @param point the point in map coordinates whose graph coordinates are required
   * @param map_size the size of the output map
   * @param target_resolution target resolution of the output map (m/pixels)
   * @return point in graph frame
   */
  static Point2D transformToGraphCoords(const floorplanGraph &graph, Point2D point, cv::Size map_size, double target_resolution)
  {
    Point2D map_centroid(map_size.width/2.0, map_size.height/2.0);
    
    // m/pixel in the current scale
    double current_resolution = graph.m_property->real_distance / graph.m_property->pixel_distance;
    auto offsetted_point = point - map_centroid; 
    auto graph_point = offsetted_point * target_resolution / graph.m_property->real_distance + graph.m_property->centroid;

    return graph_point;
  }

  /**
   * @brief get start and end of a line segment in the map frame
   * @param graph the floorplan graph
   * @param segment the line segment whose endpoints are desired
   * @param map_size the size of the output map
   * @param target_resolution target resolution of the output map (m/pixels)
   */
  static std::array<Point2D, 2> getSegmentStartEnd(const floorplanGraph &graph, const LineSegment segment, cv::Size map_size, double target_resolution)
  {
    return {
        transformToMapCoords(graph, segment.startPos, map_size, target_resolution), 
        transformToMapCoords(graph, segment.endPos,   map_size, target_resolution) 
    };
  }

  /**
   * @brief get the 2D layout of a floorplahGraph as cv::Mat
   * @param filename filename to save as
   * @param graph the floorplanGraph to be saved
   * @param resolution m/pixels
   * @param size size of the map
   * @return map of opencv mat type
   */
  static cv::Mat getGraphLayout(const floorplanGraph &graph, double target_resolution=0.55, cv::Size map_size=cv::Size(256, 128))
  {
  
    cv::Mat map(map_size, CV_8UC1, cv::Scalar(255));
  
    if (graph.m_property->real_distance == -1
        || graph.m_property->pixel_distance == -1)
    {
      std::cerr << "No scale information for graph " << graph.m_property->floorname << std::endl;
      return cv::Mat();
    }
  
    // m/pixel in the current scale
    double current_resolution = graph.m_property->real_distance / graph.m_property->pixel_distance;
  
    // check if the map is going to be clipped
    if ((graph.m_property->maxx - graph.m_property->minx) * current_resolution / target_resolution > map_size.width)
    {
      std::cerr << "Map " << graph.m_property->floorname << " clipped along x" << std::endl;
    }
    if ((graph.m_property->maxy - graph.m_property->miny) * current_resolution / target_resolution > map_size.height)
    {
      std::cerr << "Map " << graph.m_property->floorname << " clipped along y" << std::endl;
    }
  
    std::vector<LineSegment> door_segments;
  
    BGL_FORALL_VERTICES(v, graph, floorplanGraph)
    {
      auto vertex = graph[v];
      auto segments = vertex.roomLayout;
  
      for (const auto &segment: segments)
      {
        if (segment.type.compare("Portal") == 0)
        {
          door_segments.push_back(segment);
          continue;
        }
  
        auto map_start_end = getSegmentStartEnd(graph, segment, map_size, target_resolution);
  
        cv::line(
          map,
          cv::Point(map_start_end[0].x, map_start_end[0].y), cv::Point(map_start_end[1].x, map_start_end[1].y),
          0
        );
      }
    }
  
    // clear all the door segments
    for (const auto &door_segment: door_segments)
    {
      auto map_start_end = getSegmentStartEnd(graph, door_segment, map_size, target_resolution);
  
      cv::line(
        map,
        cv::Point(map_start_end[0].x, map_start_end[0].y), cv::Point(map_start_end[1].x, map_start_end[1].y),
        255
      );
    }
  
    return map;
  }

  /**
   * @brief save the 2D layout of a floorplahGraph
   * @param filename filename to save as
   * @param graph the floorplanGraph to be saved
   * @param resolution m/pixels
   * @param size size of the map
   * @return map of opencv mat type
   */
  static cv::Mat saveGraphLayoutToPNG(std::string filename, const floorplanGraph &graph, double target_resolution=0.55, cv::Size map_size=cv::Size(256, 128))
  {
    cv::Mat map =  getGraphLayout(graph, target_resolution, map_size);
    cv::imwrite(filename, map);
    return map;
  }

};
}

#endif // GRAPHFILEOPERATIONS_H
