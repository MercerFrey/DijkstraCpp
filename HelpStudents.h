#ifndef CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
#define CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>

using namespace std;


class HelpStudents{

public:
    vector < pair< pair <int,int> , int > > ways;
    int N;
    int M;
    int K;
    vector<bool> *flag;
    vector<long long int > *dist;
    vector<vector <pair<int,int>>> *arr;
    vector<vector<int>> *flagEdge;
    vector <vector<long long int>> *fifthDist;



    priority_queue <pair< int,int>, vector <pair < int, int >>, greater<pair<int,int>> > *priority_Arr; // bozuk galiba

    HelpStudents(int  N, int  M, int K, vector < pair< pair <int,int> , int > > ways);
    long long int firstStudent();
    long long int secondStudent();
    long long int thirdStudent();
    long long int fourthStudent();
    long long int fifthStudent();

    // YOU CAN ADD YOUR HELPER FUNCTIONS AND MEMBER FIELDS

};

#endif //CMPE250_ASSIGNMENT3_HELPSTUDENTS_H
