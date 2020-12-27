#include <queue>
#include <set>
#include "HelpStudents.h"
using namespace std;


HelpStudents::HelpStudents(int  N, int  M, int K, vector < pair< pair <int,int> , int > > ways) {
    this->N=N;
    this->M = M;
    this->K = K;
    this->ways = ways;
    flag= new vector<bool>(N+3);
    dist= new vector<long long int> (N+3);

    arr = new vector<vector<pair<int,int>>> (N+3);

    priority_Arr= new priority_queue <pair< int,int>, vector <pair < int, int >>, greater<pair<int,int>> > [N+3];

    for (pair<pair<int,int>,int>  item : ways){
        pair <int , int > one;

        one.first=item.second;
        one.second=item.first.second;

        (*arr)[item.first.first].push_back(one);
        priority_Arr[item.first.first].push(one);


        pair <int,int>two (item.second,item.first.first);
        priority_Arr[item.first.second].push(two);
        (*arr)[item.first.second].push_back(two);
    }
}

long long int HelpStudents::firstStudent() {

    priority_queue <pair<int,int>, vector <pair<int ,int >>, greater<pair<int,int>>> q;

    vector<pair <long long int,int >> distOfDijkstra(100005);
    for (int i =0;i< 100005; i++){
        pair <long long int , int> a (923372036854775807,i);
        distOfDijkstra[i]=a;
    }
    distOfDijkstra[1].first=0;
    q.push(distOfDijkstra[1]);


    while (!q.empty()){
        int vertex = q.top().second;
        q.pop();

        int size = (*arr)[vertex].size();
        for (int i = 0; i< size; i++ ){

            int num = (*arr)[vertex][i].second;
            int weight = (*arr)[vertex][i].first;
            if (distOfDijkstra[num].first > distOfDijkstra[vertex].first+ weight) {
                distOfDijkstra[num].first = distOfDijkstra[vertex].first + weight;
                q.push(distOfDijkstra[num]);
            }
        }
    }
    return distOfDijkstra[K].first;

}
long long int HelpStudents::secondStudent() {

    priority_queue<pair<int,int>, vector <pair<int ,int >>, greater<pair<int,int>>> q;
    dist = new vector<long long int > (N+3, 923372036854775807);
    q.push(make_pair(0,1));
    while (!q.empty()){
        int vertex = q.top().second;
        int maxEdge = q.top().first;
        q.pop();

        if (vertex == K){
            return (*dist)[K];
        }

        for ( int i =0; i<(*arr)[vertex].size();i++){
            int num = (*arr)[vertex][i].second;
            long long int numMaxEdge = (*dist)[num];
            long long int numWeight = (*arr)[vertex][i].first;
            if (numMaxEdge > maxEdge){
                (*dist)[num]= ( maxEdge>numWeight)?maxEdge:numWeight;
                pair < long long int , int > numPair;
                numPair.first = (*dist)[num];
                numPair.second= num;
                q.push(numPair);
            }
        }
    }
    return (*dist)[K];



}
long long int HelpStudents::thirdStudent() {
    queue <int> q;
    (*flag) [1] =1;
    (*dist)[1]=0;
    q.push(1);

    while (!q.empty()){
        int vertex = q.front();
        q.pop();
        int size =  (*arr)[vertex].size();
        for (int i = 0; i< size ;i++ ){
            int num = priority_Arr[vertex].top().second;
            priority_Arr[vertex].pop();
            if (!(*flag)[num]){
                (*flag)[num]=1;
                q.push(num);
                (*dist)[num]=(*dist)[vertex]+1;
            }
        }
    }
    return (*dist)[K];
}
long long int HelpStudents::fourthStudent() {
    long long int total =0;
    int vertex = 1;
    flagEdge= new vector<vector<int>> (N+3,vector<int>(N+3));


    int weight;
    int dest ;

    while (vertex!=K){
        if( priority_Arr[vertex].empty()){
            return -1;
        }
        weight = priority_Arr[vertex].top().first;
        dest = priority_Arr[vertex].top().second;
        priority_Arr[vertex].pop();

        while ((*flagEdge)[vertex][dest]==1){
            if(priority_Arr[vertex].empty()){
                return -1;
            }
            weight = priority_Arr[vertex].top().first;
            dest = priority_Arr[vertex].top().second;
            priority_Arr[vertex].pop();
        }
        (*flagEdge)[vertex][dest]=1;
        (*flagEdge)[dest][vertex]=1;
        vertex = dest;
        total += weight;
    }
    return total;
}

long long int HelpStudents::fifthStudent() {
    fifthDist= new vector<vector<long long int>>(3,vector<long long int>(N+3,923372036854775807));

    priority_queue<pair<pair<long long int,int>,int> , vector <pair<pair<long long int ,int >,int>>, greater<pair<pair<long long int ,int>,int>>> q;
    pair<pair<long long int,int>,int> vertex;
    (*fifthDist)[0][1]=0;

    vertex.first.first=0; // distance
    vertex.second=0;  // type
    vertex.first.second=1;  // num
    q.push(vertex);
    while(!q.empty()){
        int distance = q.top().first.first;
        int source=q.top().first.second;
        int type =q.top().second;
        q.pop();
        int nextType = (type+1)%3;
        vertex.second=nextType;

        for (int i=0 ; i<(*arr)[source].size() ; i++){
            int num = (*arr)[source][i].second;
            long long int weight = (*arr)[source][i].first;
            vertex.first.second=num;
            if(nextType!=0){
                if ((*fifthDist)[nextType][num] >distance + weight) {

                    (*fifthDist)[nextType][num] = distance + weight;
                    vertex.first.first = (*fifthDist)[nextType][num];
                    q.push(vertex);
                }
            }
            else {
                if ((*fifthDist)[nextType][num] > distance ){
                    (*fifthDist)[nextType][num] = distance ;

                    vertex.first.first=(*fifthDist)[nextType][num];
                    q.push(vertex);
                }
            }
        }

    }
    if ((*fifthDist)[0][K] <= (*fifthDist)[1][K] and (*fifthDist)[0][K] <= (*fifthDist)[2][K]){
        return (*fifthDist)[0][K];
    }
    else if ( (*fifthDist)[1][K] <= (*fifthDist)[2][K]){
        return (*fifthDist)[1][K];
    }
    else {
        return (*fifthDist)[2][K];
    }


}