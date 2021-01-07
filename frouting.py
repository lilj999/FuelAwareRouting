import networkx as nx
import copy
import pylab 
import numpy as np
import time
import itertools

def debug(str):
    #print(str)
    pass

def MIN_FDIST():
    return np.sqrt(2)/2

class Config:
    FuelCapacity=2

class Location():
    def __init__(self, id, x, y,al=1,be=0):
        self.id=id
        self.x=x
        self.y=y
        self.alpha=al
        self.beta=be

    def __repr__(self):
        return str('Location {}:{:.1f},{:.1f}'.format(self.id, self.x,self.y))

    # def toString(self):
    #     return str('Location {}:{:.1f},{:.1f}'.format(self.id, self.x,self.y))


    @staticmethod
    def EuclideanDistance(a, b):
        return np.sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y))

    @staticmethod
    def ManhattanDistance(a, b):
        return np.sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y))

    def GetRemainingFuel(self):
        return 0

    def GetReservedFuel(self):
        return 0

class Depot(Location):
    
    def __init__(self, *args, **kwargs):
        super(Depot, self).__init__(*args, **kwargs)
        
    def __repr__(self):
        return str('Depot {}:{:.1f},{:.1f}'.format(self.id, self.x,self.y))

    # def toString(self):
    #     return str('Depot {}:{:.1f},{:.1f}'.format(self.id, self.x,self.y))

    def GetRemainingFuel(self):
        return Config.FuelCapacity

    def GetReservedFuel(self):
        return 0.0

class Target(Location):

    def __init__(self, *args, **kwargs):
        super(Target, self).__init__(*args, **kwargs)
        self.remainingFuel=MIN_FDIST()
        self.reservedFuel=MIN_FDIST()

    def setRemainingFuel(self,fuel):
        self.remainingFuel=fuel

    def setReservedFuel(self,fuel):
        self.reservedFuel=fuel

    def resetFuelState(self,fuelLevel):
        if fuelLevel<0:
            self.reservedFuel=MIN_FDIST()
            self.remainingFuel=Config.FuelCapacity-self.reservedFuel
        else:
            self.reservedFuel=fuelLevel
            self.remainingFuel=Config.FuelCapacity- self.reservedFuel
            
    
    def __repr__(self):
        return str('Target {}:{:.1f},{:.1f} L{:.1f},R{:.1f}'.format(self.id, self.x,self.y, self.remainingFuel,self.reservedFuel))
       
    # def toString(self):
    #     return str('Target {}:{:.1f},{:.1f} L{:.1f},R{:.1f}'.format(self.id, self.x,self.y, self.remainingFuel,self.reservedFuel))

    def GetRemainingFuel(self):
        return self.remainingFuel

    def GetReservedFuel(self):
        return self.reservedFuel


class RoadGrid():

    _id=0
    @staticmethod
    def nextID():
        RoadGrid._id=RoadGrid._id+1
        return RoadGrid._id


    def __init__(self, randomdepots=0):
        self.depots=[]
        id=0
        for i in range(100):
            id=RoadGrid.nextID()
            depot=Depot(id, np.mod(i,10)+1.0,np.floor((i)/10)+1,1,0)
            self.depots.append(depot)
 
        for i in range(randomdepots):
            id=RoadGrid.nextID()
            depot=Depot(id, np.random.rand()*9+1,np.random.rand()*9+1,1,0)
            self.depots.append(depot)

        self.targets=[]
 
        self.alpha=1
        self.beta=0
        self.enet=None

    @staticmethod
    def CloneDepot(dep):
        id=RoadGrid.nextID()
        newdep=Depot(id, dep.x, dep.y,dep.alpha,dep.beta)
        return newdep

    @staticmethod
    def CloneDepots(depots):
        newdeps=[]
        for dep in depots:
            newdeps.append(RoadGrid.CloneDepot(dep))
        return newdeps

    def ResetFuelStates(self, fuelLevel=-1):
        for tar in self.targets:
            tar.resetFuelState(fuelLevel)
       
    def UpdateEdgesWithAlphaBeta(self,net):
        self.net=net

        """ edg=net.get_edge_data(src,dst)
        wei=edg['weight']
        edg['weight']=weight """
        for a, b in net.edges():
            edg=net.get_edge_data(a,b)
            dist= Location.EuclideanDistance(a,b)
            #edg['weight']=dist*self.alpha+self.beta
            edg['weight']=dist*self.alpha+b.beta
            edg['euclidean']=dist

        

    @staticmethod
    def GetEdgeWeight(net, src, dst):
        edg=net.get_edge_data(src,dst)
        wei=edg['weight']

        return wei   

    # alpha, beta+-rand(betaX)
    def setAlphaBeta(self, alpha, beta, betaX=0):
        self.alpha=alpha
        self.beta=beta

        for dep in self.depots:
            dep.beta=beta+ (np.random.rand()-0.5)*betaX*2

    def randomizeTargets(self, numTargets):
        self.targets=[]
        for i in range(numTargets+1):
            id=RoadGrid.nextID()
            tar=Target(id, np.random.rand()*9+1,np.random.rand()*9+1)
            self.targets.append(tar)

    def minimizeTargetWeights(self):
        for tar in self.targets:
            minidist=99999.0
            for d in self.depots:
                dist=Location.EuclideanDistance(tar,d)
                if dist< minidist:
                    minidist=dist
            tar.reservedFuel=minidist
    
    def calTargetsLength(self):
        total=0
        numTargets=len(self.targets)
        for i in range(numTargets-1):
            dist=Location.EuclideanDistance(self.targets[i],self.targets[i+1])
            total=total+dist
        return total    

    def calPathLength(self, path):
        total=0
        numNode=len(path)
        for i in range(numNode-1):
            dist=Location.EuclideanDistance(path[i],path[i+1])
            total=total+dist
        return total    

    def calTargetsLengthWithAlphaBeta(self):
        total=0
        numTargets=len(self.targets)
        for i in range(numTargets-1):
            dist=Location.EuclideanDistance(self.targets[i],self.targets[i+1])
            #total=total+dist*self.alpha+self.beta
            total=total+dist*self.alpha
        return total    

    def calPathLengthWithAlphaBeta(self, path):
        total=0
        numNode=len(path)
        for i in range(numNode-1):
            dist=Location.EuclideanDistance(path[i],path[i+1])
            if path[i+1] is Depot:
                total=total+dist*self.alpha+path[i+1].beta
            else:
                total=total+dist*self.alpha
        return total    

    def calPathLengthWithEuclidean(self, path):
        total=0
        numNode=len(path)
        for i in range(numNode-1):
            dist=Location.EuclideanDistance(path[i],path[i+1])
            total=total+dist
        return total    

    def calIndexedTargetLength(self, indexes):
        total=0
        numTargets=len(indexes)
        for i in range(numTargets-1):
            dist=Location.EuclideanDistance(self.targets[indexes[i]],self.targets[indexes[i+1]])
            total=total+dist

        dist=Location.EuclideanDistance(self.targets[indexes[numTargets-1]],self.targets[indexes[0]])
        total=total+dist 
        return total    

    def TspOrderedTargets(self):
        nat=range(len(self.targets))
        minlen=99999.0
        mintargets=[]
        for item in itertools.permutations(nat):
            itemlen=self.calIndexedTargetLength(item)
            if minlen>itemlen:
                minlen=itemlen
                mintargets=item
        
        tars=[]
        for i in nat:
            tars.append(self.targets[mintargets[i]])
        mintargets=tars
        return mintargets,minlen

    def showPath(self, net, path):
        pos=nx.get_node_attributes(net, 'pos')
        nx.draw(net,pos,with_labels=False, node_color='black', edge_color='red', node_size=400, alpha=0.5 )
        pylab.title('Draw Path',fontsize=15)
        path_edges = zip(path,path[1:])
        path_edges = set(path_edges)
        nx.draw_networkx_nodes(net,pos,nodelist=path,node_color='r')
        nx.draw_networkx_edges(net,pos,edgelist=path_edges,edge_color='r',width=10)
        pylab.show()

    def findBetweenTargetsforWindow(self,fuelCapacity, dnet, etargets, itarget):
        debug('SW: forWindow: itarget {}'.format( itarget))
        enet=copy.deepcopy(dnet)
        #etargets= copy.deepcopy(self.targets)
        src=etargets[itarget]
        dst=etargets[itarget+1] 
        enet.add_node(src, pos=(src.x,src.y))
        for b in enet.nodes():
            dist=Location.EuclideanDistance(src,b)
            if src is not b and dist< fuelCapacity:
                enet.add_weighted_edges_from([(src,b,dist)])

        enet.add_node(dst, pos=(dst.x,dst.y))
        for a in enet.nodes():
            dist=Location.EuclideanDistance(a,dst)
            if a is not dst and dist< fuelCapacity:
                enet.add_weighted_edges_from([(a,dst,dist)])

        self.UpdateEdgesWithAlphaBeta(enet)        

        path=[]
        try:
            while True:
                found=True
                path=nx.dijkstra_path(enet, source=src, target=dst)
                debug('SW:path from src to dst：{}'.format(path))
               
                #check dst
                dist=Location.EuclideanDistance(path[-2],path[-1])
                if path[-2].GetRemainingFuel()- (dist+ path[-1].GetReservedFuel())<0:
                    enet.remove_edge(path[-2],path[-1])
                    found=False
                    debug('SW:failed dst withdst:{:.1f} R{:.1f} L{:.1f}> {}'.format( dist, path[-2].GetRemainingFuel(),path[-1].GetReservedFuel(), fuelCapacity))
                #check src
                if len(path)>2:
                    dist=Location.EuclideanDistance(path[0],path[1])    
                    #if path[0].remainingFuel+dist >fuelCapacity:
                    if path[0].GetRemainingFuel()<dist:
                        enet.remove_edge(path[0],path[1])
                        found=False
                        debug('SW:failed src withdst:{:.1f} <{:.1f} '.format(path[0].remainingFuel, dist ))

                if found:
                    break
        except Exception:
            found=False
            distance=-1
            debug('SW: not found forWindow targets {}'.format(etargets))
            return distance,-1, [src], 0

        #srcdist=nx.dijkstra_path_length(enet, source=path[0], target=path[1])
        #dstdist=nx.dijkstra_path_length(enet, source=path[-2], target=path[-1])
        srcdist=Location.EuclideanDistance(path[0],path[1])  
        dstdist=Location.EuclideanDistance(path[-2],path[-1])  
        path[0].reservedFuel= path[1].GetReservedFuel()+ srcdist
        path[-1].remainingFuel=path[-2].GetRemainingFuel()-dstdist
        debug('SW: forWindow set {} reservedFuel  {} with next {} srcdist: {}'.format(etargets[itarget],path[0].GetReservedFuel(),path[1].GetReservedFuel(), srcdist))
        distance=nx.dijkstra_path_length(enet, source=src, target=dst)
        edistance=nx.dijkstra_path_length(enet, source=src, target=dst, weight='euclidean')
        debug('SW: forWindow targets {}'.format(etargets))
        debug('SW: forWindow return {:.1f},{},{:.1f}'.format( distance, path, dstdist))

        #self.showPath(enet,path)
        return distance,edistance, path, dstdist

    def findPathBetweenTargetsInWindow(self,fuelCapacity, dnet, etargets, itarget, winsize):
        debug('SW: InWindow: itarget {} win {}'.format(itarget, winsize))
        #enet=copy.deepcopy(dnet)

        iLast=itarget+winsize
        if iLast> len(self.targets)-1:
            iLast=len(self.targets)-1

        distance=0
        edistance=0
        path=[]
        dstdist=0
        for i in range(iLast,itarget,-1) :
            #src=self.targets[i-1]
            #dst=self.targets[i] 
            distance,edistance, path,dstdist=self.findBetweenTargetsforWindow(fuelCapacity, dnet, etargets, i-1)

        #debug('SW: InWindow return: {:.1f}, {}, {:.1f}'.format(distance, path, dstdist))
        return distance,edistance,path,dstdist



    def findPathwithSlidingWindow(self,fuelCapacity, winsize):
        start_time = time.time()
        depotNet=nx.DiGraph()

        for d in self.depots:
            depotNet.add_node(d, pos=(d.x,d.y))

        for a in depotNet.nodes():
            for b in depotNet.nodes():
                dist=Location.EuclideanDistance(a,b)
                if a is not b and dist< fuelCapacity:
                    depotNet.add_weighted_edges_from([(a,b,dist)])
        
        self.minimizeTargetWeights()
        distance=0
        edistance=0
        totalpath=[self.targets[0]]
        for i in range(len(self.targets)-1):
            mindist=99999
            minedist=99999
            minpath=[]
            minidstdist=0
            for j in range(winsize):
                etargets= copy.deepcopy(self.targets)
                dist,edist,path,dstdist=self.findPathBetweenTargetsInWindow(fuelCapacity,depotNet,etargets,i,j+1)
                if mindist>dist:
                    mindist=dist
                    minedist=edist
                    minpath=path
                    minidstdist=dstdist
             
            self.targets[i].reservedFuel= path[0].GetReservedFuel() 
            self.targets[i+1].remainingFuel= path[-1].GetRemainingFuel()
            
            distance=distance+mindist
            edistance=edistance+minedist
            totalpath=totalpath+ minpath[1:]

        runningtime = time.time() - start_time     

        
        return distance,edistance,totalpath,runningtime/(len(self.targets)-1)


    def findGRBetweenTargets(self,fuelCapacity, dnet, src, dst):

        enet=copy.deepcopy(dnet)
        enet.add_node(src, pos=(src.x,src.y))
        for b in enet.nodes():
            dist=Location.EuclideanDistance(src,b)
            if src is not b and dist< fuelCapacity:
                enet.add_weighted_edges_from([(src,b,dist)])

        enet.add_node(dst, pos=(dst.x,dst.y))
        for a in enet.nodes():
            dist=Location.EuclideanDistance(a,dst)
            if a is not dst and dist< fuelCapacity:
                enet.add_weighted_edges_from([(a,dst,dist)])
        
        self.UpdateEdgesWithAlphaBeta(enet)   
        path=[]
        try:
            while True:
                found=True
                path=nx.dijkstra_path(enet, source=src, target=dst)
                debug('GR:path from src to dst：'.format(path))
               
                #check dst
                dist=Location.EuclideanDistance(path[-2],path[-1])
                if path[-2].GetRemainingFuel()- (dist+ path[-1].GetReservedFuel())<0:
                    enet.remove_edge(path[-2],path[-1])
                    found=False
                    debug('GR:failed dst withdst:{:.1f} +{:.1f}> {}'.format( dist, path[-1].reservedFuel, fuelCapacity))
                #check src
                if len(path)>2:
                    dist=Location.EuclideanDistance(path[0],path[1])    
                    if path[0].remainingFuel<dist:
                        enet.remove_edge(path[0],path[1])
                        found=False
                        debug('GR:failed src withdst:{:.1f} <{:.1f} '.format(path[0].remainingFuel, dist ))

                if found:
                    break
        except Exception:
            found=False
            distance=-1
            return distance, -1, [src], -1

        srcdist=Location.EuclideanDistance(path[0],path[1])  
        #dstdist=Location.EuclideanDistance(path[-2],path[-1])
        distance=nx.dijkstra_path_length(enet, source=src, target=dst)
        edistance=nx.dijkstra_path_length(enet, source=src, target=dst,weight='euclidean')
        debug('GR:Found dist between src and dst：{:.1f}'.format( distance))

        return distance,edistance,path,srcdist


    def findPathwithGreedyRouting(self,fuelCapacity):
        start_time = time.time()
        depotNet=nx.DiGraph()

        for d in self.depots:
            depotNet.add_node(d, pos=(d.x,d.y))

        for a in depotNet.nodes():
            for b in depotNet.nodes():
                dist=Location.EuclideanDistance(a,b)
                if a is not b and dist< fuelCapacity:
                    depotNet.add_weighted_edges_from([(a,b,dist)])
        
        distance=0
        edistance=0
        path=[self.targets[-1]]
        for i in range(len(self.targets)-1, 0,-1):
            dist,edist,subpath, srcdist=self.findGRBetweenTargets(fuelCapacity,depotNet,self.targets[i-1],self.targets[i])
            distance=distance+dist
            edistance=edistance+edist
            path=subpath[:-1] + path
            self.targets[i-1].reservedFuel= subpath[1].GetReservedFuel()+ srcdist#fuelCapacity-dstdist

        runningtime = time.time() - start_time  
        return distance,edistance,path,runningtime


    def findPathwithKhuller(self,fuelCapacity):
        start_time = time.time()
        depotNet=nx.DiGraph()

        deps=self.depots
        src=self.targets[0]
        dst=self.targets[-1]

        for i in range(len(self.targets)-1):
            for d in deps:
                depotNet.add_node(d, pos=(d.x,d.y))
            for a in deps:
                for b in deps:
                    dist=Location.EuclideanDistance(a,b)
                    if a is not b and dist< fuelCapacity:
                        depotNet.add_weighted_edges_from([(a,b,dist)])
            if i==0:
                depotNet.add_node(self.targets[i], pos=(self.targets[i].x,self.targets[i].y))            
            for b in deps:
                dist=Location.EuclideanDistance(self.targets[i],b)
                if self.targets[i] is not b and dist< fuelCapacity-self.targets[i].remainingFuel:
                    depotNet.add_weighted_edges_from([(self.targets[i],b,dist)])
            
            depotNet.add_node(self.targets[i+1], pos=(self.targets[i+1].x,self.targets[i+1].y))     
            for a in deps:
                dist=Location.EuclideanDistance(a, self.targets[i+1])
                if a  is not self.targets[i+1] and dist< fuelCapacity-self.targets[i+1].reservedFuel:
                    depotNet.add_weighted_edges_from([(a, self.targets[i+1],dist)])

            deps=RoadGrid.CloneDepots(self.depots)


        self.UpdateEdgesWithAlphaBeta(depotNet)   
        path=[]
        try:
            path=nx.dijkstra_path(depotNet, source=src, target=dst)
            debug('Khuller:path from src to dst ：{}'.format( path))
            distance=nx.dijkstra_path_length(depotNet, source=src, target=dst)
            edistance=nx.dijkstra_path_length(depotNet, source=src, target=dst,weight='euclidean')
            debug('Khuller:Found dist between src and dst：{:.1f}'.format(distance))
        except Exception:
            found=False
            distance=0
            edistance=0
                           
        runningtime = time.time() - start_time                           
        return distance, edistance,path, runningtime

    def findPathwithKhullerTour(self,fuelCapacity):
        start_time = time.time()
        depotNet=nx.DiGraph()

        deps=self.depots
        src=self.targets[0]
        dst=self.targets[-1]

        for i in range(len(self.targets)-1):
            for d in deps:
                depotNet.add_node(d, pos=(d.x,d.y))
            for a in deps:
                for b in deps:
                    dist=Location.EuclideanDistance(a,b)
                    if a is not b and dist< fuelCapacity:
                        depotNet.add_weighted_edges_from([(a,b,dist)])
            if i==0:
                depotNet.add_node(self.targets[i], pos=(self.targets[i].x,self.targets[i].y))            
            for b in deps:
                dist=Location.EuclideanDistance(self.targets[i],b)
                if self.targets[i] is not b and dist< fuelCapacity-self.targets[i].remainingFuel:
                    depotNet.add_weighted_edges_from([(self.targets[i],b,dist)])
            
            depotNet.add_node(self.targets[i+1], pos=(self.targets[i+1].x,self.targets[i+1].y))     
            for a in deps:
                dist=Location.EuclideanDistance(a, self.targets[i+1])
                if a  is not self.targets[i+1] and dist< fuelCapacity-self.targets[i+1].reservedFuel:
                    depotNet.add_weighted_edges_from([(a, self.targets[i+1],dist)])

            deps=RoadGrid.CloneDepots(self.depots)


        self.UpdateEdgesWithAlphaBeta(depotNet)   
        path=[]
        try:
            path=nx.dijkstra_path(depotNet, source=src, target=dst)
            debug('Khuller:path from src to dst ：{}'.format( path))
            distance=nx.dijkstra_path_length(depotNet, source=src, target=dst)
            edistance=nx.dijkstra_path_length(depotNet, source=src, target=dst,weight='euclidean')
            debug('Khuller:Found dist between src and dst：{:.1f}'.format(distance))
        except Exception:
            found=False
            distance=0
                           
        runningtime = time.time() - start_time                           
        return distance, edistance,path, runningtime
    

    @staticmethod
    def TestWindow(rg):
        #np.random.seed(3)
        

        for i in range(2):
            Config.FuelCapacity=i*5+1.5

            depotNet=nx.DiGraph()
            for d in rg.depots:
                depotNet.add_node(d, pos=(d.x,d.y))

            for a in depotNet.nodes():
                for b in depotNet.nodes():
                    dist=Location.EuclideanDistance(a,b)
                    if a is not b and dist< Config.FuelCapacity:
                        depotNet.add_weighted_edges_from([(a,b,dist)])

            for k in range(len(rg.targets)-1):
                for j in range(10,0, -1):
                    np.random.seed(3)
                    fuelLeft=1.5/2
                    rg.ResetFuelStates(fuelLeft)
                    etargets= copy.deepcopy(rg.targets)
                    dist,edist,path,dstdist=rg.findPathBetweenTargetsInWindow(Config.FuelCapacity,depotNet,etargets,k,j)
                    #print('Khuller:{} Left:{} path:{}, dist:{:.1f},time:{}\n'.format(fuelcapacity,fuelLeft, totalpath,distance,runtime))
                    print('Window: Cap:{} Left:{} iTar:{} Win:{}, dist:{:.1f}'.format(Config.FuelCapacity,fuelLeft,k,j,dist))

            # np.random.seed(3)
            # fuelLeft=1.5
            # k=3
            # j=1
            # rg.ResetFuelStates(fuelLeft)
            # etargets= copy.deepcopy(rg.targets)
            # dist,path,dstdist=rg.findPathBetweenTargetsInWindow(Config.FuelCapacity,depotNet,etargets,k,j)
            # #print('Khuller:{} Left:{} path:{}, dist:{:.1f},time:{}\n'.format(fuelcapacity,fuelLeft, totalpath,distance,runtime))
            # print('Window: Cap:{} Left:{} iTar:{} Win:{}, dist:{:.1f}'.format(Config.FuelCapacity,fuelLeft,k,j,dist))
            # j=2
            # rg.ResetFuelStates(fuelLeft)
            # etargets= copy.deepcopy(rg.targets)
            # dist,path,dstdist=rg.findPathBetweenTargetsInWindow(Config.FuelCapacity,depotNet,etargets,k,j)
            # #print('Khuller:{} Left:{} path:{}, dist:{:.1f},time:{}\n'.format(fuelcapacity,fuelLeft, totalpath,distance,runtime))
            # print('Window: Cap:{} Left:{} iTar:{} Win:{}, dist:{:.1f}'.format(Config.FuelCapacity,fuelLeft,k,j,dist))
class TestUnit:

    @staticmethod
    def Validating():
        Config.FuelCapacity=1.5#MIN_FDIST()*2
        Config.FuelLeft=Config.FuelCapacity/2

        np.random.seed(3)

        rg=RoadGrid(0)
        #rg.setAlphaBeta(1, 1)
        rg.setAlphaBeta(3, 1)
        rg.randomizeTargets(10)
        #rg.setAlphaBeta(1,0)
        reflength=rg.calTargetsLengthWithAlphaBeta()

        #RoadGrid.TestKhuller(rg)
        #RoadGrid.TestWindow(rg)

        for winsize in range(1,5):
            rg.ResetFuelStates(Config.FuelLeft)
            distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
            print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))

        rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        distance, edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
    
    @staticmethod
    def TestWindows():
        Config.FuelCapacity=1.5#MIN_FDIST()*2
        Config.FuelLeft=Config.FuelCapacity/2

        np.random.seed(3)

        rg=RoadGrid(0)
        #rg.setAlphaBeta(1, 1)
        rg.setAlphaBeta(3, 1)
        rg.randomizeTargets(10)
        #rg.setAlphaBeta(1,0)
        reflength=rg.calTargetsLengthWithAlphaBeta()

        RoadGrid.TestWindow(rg)


    @staticmethod
    def calculateConfidenceBoundary(lstNumber):
        nonZeros=[]
        for num in lstNumber:
            if num>0:
                nonZeros.append(num)
        
        total=0.0
        count=len(nonZeros)
        if count==0:
            return 0,0,0,0
        for num in nonZeros:
            total=total+num
        mean= total/count

        squaredDifferenceSum=0
        for num in nonZeros:
            total=total+num
            squaredDifferenceSum = squaredDifferenceSum+(num - mean) * (num - mean)

        variance = squaredDifferenceSum / count
        standardDeviation = np.sqrt(variance)

        # value for 95% confidence interval, source: https://en.wikipedia.org/wiki/Confidence_interval#Basic_Steps

        # //        C	z*
        # //        99%	2.576
        # //        98%	2.326
        # //        95%	1.96
        # //        90%	1.645
        confidenceLevel = 1.645 #90%
        temp = confidenceLevel * standardDeviation / np.sqrt(count)
        return mean, mean - temp, mean + temp, temp
        
    @staticmethod
    def calculateConfidenceforTable(tabData):

        means=[]
        temps=[]
        rowNum=len(tabData)
        colNum=len(tabData[0])
        for j in range(colNum):
            col=[i[j] for i in tabData]
            mean, lb, ub, tmp=TestUnit.calculateConfidenceBoundary(col)
            means.append(mean)
            temps.append(tmp)
        
        return means,  temps

    @staticmethod
    def TestOne(seed=0, capacity=1.5, fleft=0.75, alpha=1, beta=0, numtargets=10):
        Config.FuelCapacity=capacity#MIN_FDIST()*2
        Config.FuelLeft=fleft

        np.random.seed(seed)

        rg=RoadGrid(0)
        rg.setAlphaBeta(alpha, beta,0)
        rg.randomizeTargets(numtargets)
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()

        listDist=[]
        listeDist=[]
        listRuntime=[]
        for winsize in range(1,4):
            rg.ResetFuelStates(Config.FuelLeft)
            distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
            listDist.append(distance/reflength)
            listeDist.append(edistance/ereflength)
            listRuntime.append(runtime)
            #print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        #print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)

        rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        distance, edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        #print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)

        return listDist, listeDist, listRuntime
    
    @staticmethod
    def GetNodePositions(net, path):
        pos=dict()#nx.get_node_attributes(net, 'pos')

        dep=[]
        tar=[]
        for e in path:
            pos[e]=(e.x,e.y)
            if isinstance(e, Target):
                tar.append(e)
            if isinstance(e, Depot):
                dep.append(e)

        return pos,dep, tar

    @staticmethod
    def GetTargetLabels(net, targets):
        label=dict()#nx.get_node_attributes(net, 'pos')

        for i in range(len(targets)):
            label[targets[i]]='p{}'.format(i)

        return label

    @staticmethod
    def DrawTargets(net, targets, color):
        pos,dep,tar=TestUnit.GetNodePositions(net,targets)
        lb=TestUnit.GetTargetLabels(net,targets)
        #nx.draw_networkx_nodes(net,pos,nodelist=targets,node_color=color)
        nx.draw_networkx_nodes(net,pos,nodelist=targets,node_color=color)
        nx.draw_networkx_labels(net,pos,labels=lb)
        pylab.show()
        pylab.legend()

    @staticmethod
    def DrawPath(net, path, color, st, first):
        if first:
            pos=nx.get_node_attributes(net, 'pos')
            nx.draw(net,pos,with_labels=False, node_color='lightgray', node_shape ='^',edge_color='lightgray', node_size=400, alpha=0.5 )
            pylab.title('Draw All Paths',fontsize=15)

        if len(path) >2:
            path_edges = zip(path,path[1:])
            path_edges = set(path_edges)
            #pos=nx.get_node_attributes(net, 'pos')
            pos,dep,tar=TestUnit.GetNodePositions(net,path)
            nx.draw_networkx_nodes(net,pos,nodelist=dep,node_shape ='^',node_color=color)
            nx.draw_networkx_nodes(net,pos,nodelist=tar,node_shape ='o',node_color=color)
            nx.draw_networkx_edges(net,pos,alpha=0.5, edgelist=path_edges,edge_color=color,sytle='dashdot',label='good', width=2)
        
        

    @staticmethod
    def TestAlphaBeta(seed=0, capacity=1.5, fleft=0.75,  numtargets=10):
        Config.FuelCapacity=capacity#MIN_FDIST()*2
        Config.FuelLeft=fleft

        np.random.seed(seed)

        rg=RoadGrid(0)
        rg.randomizeTargets(numtargets)

        drawfirst=True
        
        # path_edges = zip(path,path[1:])
        # path_edges = set(path_edges)
        # nx.draw_networkx_nodes(net,pos,nodelist=path,node_color='r')
        # nx.draw_networkx_edges(net,pos,edgelist=path_edges,edge_color='r',width=10)
        # pylab.show()

        listDist=[]
        listeDist=[]
        listRuntime=[]

        alpha=1
        beta=0
        rg.setAlphaBeta(alpha, beta, beta/2.0)
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()
        #for winsize in range(1,4):
        winsize=2
        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)

        if drawfirst:
            drawfirst=False
            TestUnit.DrawPath(rg.net, totalpath, 'y', 'solid', True)
        else:
            TestUnit.DrawPath(rg.net, totalpath, 'y', 'solid', False)
            #print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        # rg.ResetFuelStates(Config.FuelLeft)
        # distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        # #print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'r', False, False)
       
        # rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        # distance, edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        # #print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'r', False, False)


        alpha=1
        beta=1
        rg.setAlphaBeta(alpha, beta,beta/2.0)
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()
        #for winsize in range(1,4):
        winsize=2
        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)
        TestUnit.DrawPath(rg.net, totalpath, 'g', 'dashed',False)
            #print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        # rg.ResetFuelStates(Config.FuelLeft)
        # distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        # #print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'g', False, False)

        # rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        # distance, edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        # #print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'g', False, False)

        alpha=3
        beta=1
        rg.setAlphaBeta(alpha, beta,beta/2.0)
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()
        #for winsize in range(1,4):
        winsize=2
        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)
        TestUnit.DrawPath(rg.net, totalpath, 'b', 'dotted',False)
        TestUnit.DrawTargets(rg.net, rg.targets, 'r')
            #print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        # rg.ResetFuelStates(Config.FuelLeft)
        # distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        # #print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'b', False, False)

        # rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        # distance, edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        # #print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
        # listDist.append(distance/reflength)
        # listeDist.append(edistance/ereflength)
        # listRuntime.append(runtime)
        # TestUnit.DrawPath(rg.net, totalpath, 'b', False, True)

        return listDist, listeDist, listRuntime

    @staticmethod
    def TestTen(capacity=1.5, fleft=0.75, alpha=1, beta=0, numtargets=10):
        dists=[]
        edists=[]
        runs=[]
        for seed in range(5):
            dist,edist, runtime =test.TestOne(seed+121,capacity,fleft,alpha,beta,numtargets)
            dists.append(dist)
            edists.append(edist)
            runs.append(runtime)

        meanDist,tempDist= TestUnit.calculateConfidenceforTable(dists)
        meaneDist,tempeDist= TestUnit.calculateConfidenceforTable(dists)
        meanRun,tempRun= TestUnit.calculateConfidenceforTable(runs)
        return meanDist,tempDist,meaneDist,tempeDist,meanRun, tempRun


    @staticmethod
    def TestKhullerOne(seed, capacity=1.5, alpha=1, beta=0, numtargets=10):
        Config.FuelCapacity=capacity#MIN_FDIST()*2
        #Config.FuelLeft=fleft

        np.random.seed(seed)
        rg=RoadGrid(0)
        rg.setAlphaBeta(alpha, beta)
        rg.randomizeTargets(numtargets)
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()

        listDist=[]
        listeDist=[]
        listRuntime=[]
        for j in range(11):
            fleft=capacity/10.0*j
            rg.ResetFuelStates(fleft)
            distance,edistance, totalpath,runtime =rg.findPathwithKhuller(capacity)
            listDist.append(distance/reflength)
            listeDist.append(edistance/ereflength)
            listRuntime.append(runtime)

        #print('Khuller:{} Left:{} path:{}, dist:{:.1f},time:{}\n'.format(fuelcapacity,fuelLeft, totalpath,distance,runtime))
        #print('Khuller:{} Left:{} path:, dist:{:.1f},time:{}'.format(fuelcapacity,fuelLeft,distance,runtime))
      
        return listDist,listeDist,listRuntime


    @staticmethod
    def TestKhullerLeft(capacity=1.5, alpha=1, beta=0, numtargets=10):
        dists=[]
        edists=[]
        runs=[]
        for seed in range(5):
            dist, edist,runtime=TestUnit.TestKhullerOne(seed+121,capacity,alpha,beta,numtargets)
            dists.append(dist)
            edists.append(edist)
            runs.append(runtime)

        meanDist,tempDist= TestUnit.calculateConfidenceforTable(dists)
        meanRun,tempRun= TestUnit.calculateConfidenceforTable(runs)
        return meanDist,tempDist,meanRun, tempRun
    
    @staticmethod
    def TestTourOne(seed=0, capacity=1.5, fleft=0.75, alpha=1, beta=0, numtargets=10):
        Config.FuelCapacity=capacity#MIN_FDIST()*2
        Config.FuelLeft=fleft

        np.random.seed(seed)

        rg=RoadGrid(0)
        rg.setAlphaBeta(alpha, beta)
        rg.randomizeTargets(numtargets)
        rg.targets, tarlen=rg.TspOrderedTargets()
        reflength=rg.calTargetsLengthWithAlphaBeta()
        ereflength=rg.calTargetsLength()

        listDist=[]
        listeDist=[]
        listRuntime=[]
        for winsize in range(1,4):
            rg.ResetFuelStates(Config.FuelLeft)
            distance, edistance, totalpath, runtime=rg.findPathwithSlidingWindow(Config.FuelCapacity,winsize)
            listDist.append(distance/reflength)
            listeDist.append(edistance/ereflength)
            listRuntime.append(runtime)
            #print('Path of SlidingWindow winsize{}:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(winsize,totalpath,distance,distance/reflength,runtime))

        rg.ResetFuelStates(Config.FuelLeft)
        distance, edistance, totalpath,runtime =rg.findPathwithGreedyRouting(Config.FuelCapacity)
        #print('Path of AwareRouting:{}, dist:{:.1f},Ndist:{:.1f},time:{}\n'.format(totalpath,distance,distance/reflength,runtime))
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)

        rg.ResetFuelStates(Config.FuelLeft)#rg.ResetFuelStates()
        distance,edistance, totalpath, runtime=rg.findPathwithKhuller(Config.FuelCapacity)
        #print('Path of Khuller:{}, dist:{:.1f}, Ndist:{:.1f}, time:{}\n'.format(totalpath,distance, distance/reflength,runtime))
        listDist.append(distance/reflength)
        listeDist.append(edistance/ereflength)
        listRuntime.append(runtime)

        return listDist,listeDist, listRuntime

    @staticmethod
    def TestTourTen(capacity=1.5, fleft=0.75, alpha=1, beta=0, numtargets=10):
        dists=[]
        edists=[]
        runs=[]
        for seed in range(5):
            dist, edist,runtime=test.TestTourOne(seed+121,capacity,fleft,alpha,beta,numtargets)
            dists.append(dist)
            edists.append(edist)
            runs.append(runtime)

        meanDist,tempDist= TestUnit.calculateConfidenceforTable(dists)
        meaneDist,tempeDist= TestUnit.calculateConfidenceforTable(edists)
        meanRun,tempRun= TestUnit.calculateConfidenceforTable(runs)
        return meanDist,tempDist,meaneDist,tempeDist,meanRun, tempRun

if __name__ == "__main__":
    test=TestUnit()

    #test.TestWindows()
    # seed=6
    # for i in range(5):
    #     cap= MIN_FDIST()*2+i*0.1
    #     flef=cap/2#MIN_FDIST()
    #     dist, runtime=test.TestOne(seed,cap,flef,1,0)
    #     print('ab10test {}:'.format(cap), dist,  runtime)

    # for i in range(5):
    #     cap= MIN_FDIST()*2+i*0.1
    #     flef=cap/2#MIN_FDIST()
    #     dist, runtime=test.TestOne(seed,cap,flef,1,1)
    #     print('ab11test {}:'.format(cap), dist,  runtime)

    # for i in range(5):
    #     cap= MIN_FDIST()*2+i*0.1
    #     flef=cap/2#MIN_FDIST()
    #     dist, runtime=test.TestOne(seed,cap,flef,1,3)
    #     print('ab31test {}:'.format(cap), dist,  runtime)
    
    
    #TEST fuel, delay, joint
    # for i in range(5):
    #     cap=MIN_FDIST()*2+i*0.1
    #     flef=cap/2
    #     dist, dx, runtime,rx=test.TestTen(cap,flef,1,0)
    #     print('ab10 {:.2f}'.format(cap), dist, dx, runtime, rx)

    # for i in range(5):
    #     cap=MIN_FDIST()*2+i*0.1
    #     flef=cap/2
    #     dist, dx, runtime,rx=test.TestTen(cap,flef,1,1)
    #     print('ab11 {:.2f}'.format(cap),dist, dx, runtime, rx)

    # for i in range(5):
    #     cap=MIN_FDIST()*2+i*0.1
    #     flef=cap/2
    #     dist, dx, runtime,rx=test.TestTen(cap,flef,3,1)
    #     print('ab31 {:.2f}'.format(cap), dist, dx, runtime, rx)
    
    # TEST targets
    # # seed=6
    # for i in range(0,100,10):
    #     cap= 1.61
    #     flef=cap/2#MIN_FDIST()
    #     dist, runtime=test.TestOne(1,cap,flef,3,1,i+1)
    #     print('targets {}:'.format(i), dist,  runtime)

    #TEST Khuller
    # seed=1
    # for i in range(10):
    #     cap=MIN_FDIST()*2+i*0.1
    #     # dist, runtime=test.TestKhullerOne(i+1, cap,1,0)
    #     # print('khuller {:.2f}'.format(cap), dist,  runtime)
    #     dist, dx, runtime,rx=test.TestKhullerLeft(cap,1,0)
    #     print('khuller {:.2f}'.format(cap), dist, dx, runtime, rx)


    #TEST fuel with tour
    # for i in range(5):
    #     cap=MIN_FDIST()*2+i*0.1
    #     flef=cap/2
    #     dist, dx, runtime,rx=test.TestTourTen(cap,flef,1,0,8)
    #     print('tour {:.2f}'.format(cap), dist, dx, runtime, rx)


    #TEST alpha beta
    for i in range(5):
        cap=MIN_FDIST()*2+i*0.1
        flef=cap/2
        listDist, listeDist, listRuntime=test.TestAlphaBeta(121,cap,flef,10)
        print('alpahbeta {:.2f}'.format(cap), listDist, listeDist, listRuntime)