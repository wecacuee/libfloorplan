/**
  LibFloorplan
  Author: Alper Aydemir. Jan, 2012.
*/

#include "GraphDatabase.hpp"
#include "GraphFileOperations.hpp"
#include "GraphUtils.hpp"

#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/graph/adj_list_serialize.hpp>

using namespace std;
namespace floorplan {

//----------------------------------------------------------------
GraphDatabase::GraphDatabase()
{
}


//----------------------------------------------------------------
void GraphDatabase::loadGraphs(string sDir, string rootNodeName, int iLimit, bool append){
    if (!append)
        _graphs = vector<floorplanGraph>();
    _graphProperties = GraphFileOperations::loadAllGraphsInFolder(sDir, _graphs, rootNodeName, iLimit);
    updateExtent();
}


//----------------------------------------------------------------
template<class Archive>
void GraphDatabase::serialize(Archive &ar, const unsigned int version)
{

    ar & _graphs;
    ar & _graphProperties;
}


//----------------------------------------------------------------
void GraphDatabase::Save(string sFilename)
{
    ofstream ofs(sFilename.c_str(), ios::binary);

    boost::archive::binary_oarchive oa(ofs);
    oa << (*this);
    ofs.close();
}

//----------------------------------------------------------------
void GraphDatabase::Load(string sFilename)
{
    std::ifstream ifs(sFilename.c_str(), ios::binary);


    boost::archive::binary_iarchive oa(ifs);
    oa >> (*this);

    ifs.close();
}

//----------------------------------------------------------------
int GraphDatabase::replaceCategory(std::string oldCategory, std::string newCategory){
    cout << "Replacing category: " << oldCategory << "for: " << newCategory << endl;
    int nodesAffected = 0;
    for (unsigned int i=0; i < _graphs.size(); i++){
        BGL_FORALL_VERTICES(v, _graphs[i], floorplanGraph){
            if (_graphs[i][v].category.compare(oldCategory) == 0){
                _graphs[i][v].category = newCategory;
                nodesAffected++;
            }
        }
    }
    return nodesAffected;
}

//----------------------------------------------------------------
int  GraphDatabase::mergeCentralNodes(int degreeThreshold, std::string newCategory){
    int nodesAffected = 0;
    for (unsigned int i=0; i < _graphs.size(); i++){
        BGL_FORALL_VERTICES(v, _graphs[i], floorplanGraph){
            if (in_degree(v, _graphs[i]) >= degreeThreshold){
                _graphs[i][v].category = newCategory;
                nodesAffected++;
            }
        }
    }
    return nodesAffected;
}

//----------------------------------------------------------------
int  GraphDatabase::removeIsolatedVertices(){

    for (unsigned int i=0; i < _graphs.size(); i++){
        vector<string> verticesToRemove;
        BGL_FORALL_VERTICES(v, _graphs[i], floorplanGraph){
            if (in_degree(v, _graphs[i]) == 0){
                verticesToRemove.push_back(_graphs[i][v].vertex_id);
            }
        }

        for (unsigned int j=0; j < verticesToRemove.size(); j++){
            floorplanGraph tmpGraph;
            GraphUtils::removeVertex(GraphUtils::doesVertexExists(verticesToRemove[j], _graphs[i]).second, _graphs[i], tmpGraph);
            _graphs[i] = tmpGraph;
        }
    }
}

//----------------------------------------------------------------
int  GraphDatabase::removeGraphsSmallerThan(int sizeThreshold){
    int graphsRemoved = 0;
    vector<floorplanGraph>::iterator it = _graphs.begin();
    int k = 0;
    for (; it != _graphs.end(); ++it){
        if(num_vertices(*it) < sizeThreshold){
            _graphs.erase(it);
            _graphProperties.erase(_graphProperties.begin()+k);
            graphsRemoved++;
            k++;
        }
    }
    return graphsRemoved;
}

//----------------------------------------------------------------
void GraphDatabase::removeCategoriesBasedonFrequency(int freqThreshold){
    map<string, int> categoryCount;
    vector<string> labels;

    /* Fill up the labels and categoryCount container */
    for (unsigned int i = 0; i < _graphs.size(); i++){
        BGL_FORALL_VERTICES(v, _graphs[i], floorplanGraph){
            vector<string>::iterator it = find(labels.begin(), labels.end(), _graphs[i][v].category);
            if (it == labels.end()){
                  labels.push_back(_graphs[i][v].category);
                  categoryCount[_graphs[i][v].category] = 0;
            }
        }
    }

    // count all the categories in the database
    for (unsigned int i=0; i < _graphs.size(); i++){
        BGL_FORALL_VERTICES(v, _graphs[i], floorplanGraph){
                categoryCount[_graphs[i][v].category]++;
            }
        }
    // Determine categories to remove
    vector<string> categoriesToRemove;
    map<string, int>::iterator it;
    for (it = categoryCount.begin(); it != categoryCount.end(); ++it){
        if (it->second < freqThreshold){
            cout << "Category: " << it->first << "has: " << it->second <<endl;
            categoriesToRemove.push_back(it->first);
        }
    }

    // Remove those categories from each graph in the database
    vector<int> indicesToBeRemoved;
    for (unsigned int i =0; i < categoriesToRemove.size(); i++){
        cout << "Removing category " << categoriesToRemove[i] << endl;
        for (unsigned int j = 0; j < _graphs.size(); j++){
            floorplanGraph tmpGraph = _graphs[j];
           bool toremove= GraphUtils::removeCategory(categoriesToRemove[i],tmpGraph, _graphs[j]);
           if(toremove){
               indicesToBeRemoved.push_back(j);
           }

        }
    }

//    for (unsigned int i =0; i < indicesToBeRemoved.size(); i++){
//        _graphs.erase(_graphs.begin()+indicesToBeRemoved[i]);
//        _graphProperties.erase(_graphProperties.begin()+indicesToBeRemoved[i]);
//    }

}

//----------------------------------------------------------------
void GraphDatabase::Init(){
    cout << "Preparing graph database" << endl;
    replaceCategory("LAB SV", "RS LAB");
    replaceCategory("RES LO", "RS LAB");
    replaceCategory("LAB", "RS LAB");

    replaceCategory("F LAV", "BATH");
    replaceCategory("M LAV", "BATH");
    replaceCategory("P LAV", "BATH");

    replaceCategory("OFF SV", "OFF");
    replaceCategory("FOODSV", "FOOD");

    replaceCategory("P CIRC", "CORR");

    replaceCategory("CLA SV", "CLASS");


    mergeCentralNodes(3, "CORR");
    removeGraphsSmallerThan(5);
    removeCategoriesBasedonFrequency(100);
}

void GraphDatabase::updateExtent() {
    for (auto &graph: _graphs) {
        std::array<double, 4> extent = {
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::min(),
            std::numeric_limits<double>::min()
        };

        size_t num_spaces = 0;
        BGL_FORALL_VERTICES(v, graph, floorplanGraph) {
            auto vertex = graph[v];
            
            if (vertex.minx < extent[0]) {
                extent[0] = vertex.minx;
            }
            if (vertex.miny < extent[1]) {
                extent[1] = vertex.miny;
            }
            if (vertex.maxx > extent[2]) {
                extent[2] = vertex.maxx;
            }
            if (vertex.maxy > extent[3]) {
                extent[3] = vertex.maxy;
            }

            graph.m_property->centroid.x += vertex.centroid.x;
            graph.m_property->centroid.y += vertex.centroid.y;
            num_spaces++;
        }

        graph.m_property->minx = extent[0];
        graph.m_property->miny = extent[1];
        graph.m_property->maxx = extent[2];
        graph.m_property->maxy = extent[3];

        // centroid is just in the middle of min_x
        graph.m_property->centroid.x = (graph.m_property->minx + graph.m_property->maxx)/2;
        graph.m_property->centroid.y = (graph.m_property->miny + graph.m_property->maxy)/2;
    }
}

}
