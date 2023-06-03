using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Jobs;
using Unity.Jobs;
using Random = UnityEngine.Random;

public class Grid3D : MonoBehaviour
{
    public int cells_x, cells_y, cells_z;
    public float cell_size = 1.0f;
    public int nbBubulle;
    public int maxIterProjection = 5;
    public int maxIterPoisson = 5;
    //public Vector3[,,] velocity;
    public Vector3[] velocity;
    private NativeArray<Vector3> velocity_j;
    //public float[,,] pressures;
    public float[] pressures;
    private NativeArray<float> pressure_j;
    public GameObject bubullePrefab;
    public List<GameObject> bubulles;
    private float minx, maxx, miny, maxy, minz, maxz;
    //private float[,,] divergence;
    private float[] divergence;
    private NativeArray<float> divergence_j;
    private NativeArray<float> newPressure;
    private Vector3[] bubullesPos;
    private Vector3[] bubullesVel;
    private NativeArray<Vector3> bubullesPos_j;
    private NativeArray<Vector3> bubullesVel_j;
    //Job advection
    private JobHandle AdvectioJobHandle;
    UpdateAdvectionJob _updateAdvectionJob;
    //Job projection
    private JobHandle ProjetionJobHandle;
    UpdateProjectionJob _updateProjectionJob;
    int getIndex(int x, int y, int z)
    {
        return x * (cells_y * cells_z) + y * cells_z + z;
    }
    void Awake()
    {
        //Init Grid et bubulles
        Vector3 gridOrg = transform.position;
        velocity = new Vector3[cells_x*cells_y*cells_z];
        pressures = new float[cells_x*cells_y*cells_z];
        divergence = new float[cells_x*cells_y*cells_z];
        float[] initPress = new float[cells_x*cells_y*cells_z];
        bubullesPos = new Vector3[nbBubulle];
        bubullesVel = new Vector3[nbBubulle];
        //Init grid avec les cells à 0 partout
        for (int i = 0; i < cells_x; i++)
        {
            for (int j = 0; j < cells_y; j++)
            {
                for (int k = 0; k < cells_z; k++)
                {
                    //velocity[i, j, k] = Vector3.zero;
                    initPress[i * (cells_y * cells_z) + j * cells_z + k] = 0.0f;
                    velocity[i*(cells_y*cells_z)+j*cells_z+k] = new Vector3(Random.Range(-1,1)*2,
                        Random.Range(-1,1)*2,
                        Random.Range(-1,1)*2);
                    pressures[i*(cells_y*cells_z)+j*cells_z+k] = j+1;
                    divergence[i*(cells_y*cells_z)+j*cells_z+k] = 0.0f;
                }
            }
        }
        //Init gridOrg pour les calculs de position
        minx = gridOrg.x;
        miny = gridOrg.y;
        minz = gridOrg.z;
        maxx = gridOrg.x + cells_x;
        maxy = gridOrg.y + cells_y;
        maxz = gridOrg.z + cells_z;
        //Init bubulles et les mettre dans la liste
        bubulles = new List<GameObject>();
        for (int i = 0; i < nbBubulle; i++)
        {
            Vector3 pos = new Vector3(Random.Range(0 + gridOrg.x, cells_x + gridOrg.x),
                Random.Range(0 + gridOrg.y, cells_y + gridOrg.y),
                Random.Range(0 + gridOrg.z, cells_z + gridOrg.z));
            GameObject bubulle = Instantiate(bubullePrefab, pos, Quaternion.identity);
            bubulles.Add(bubulle);
            bubulle.transform.parent = transform;
            bubulle.GetComponent<Bubulle>().velocity = new Vector3(Random.Range(-0.1f,0.1f),
                Random.Range(-0.1f,0.1f),
                Random.Range(-0.1f,0.1f));
            bubulle.name = "bubulle" + i;
            bubullesPos[i] = bubulle.transform.position;
            bubullesVel[i] = bubulle.GetComponent<Rigidbody>().velocity;
        }

        velocity_j = new NativeArray<Vector3>(velocity, Allocator.Persistent);
        pressure_j = new NativeArray<float>(pressures, Allocator.Persistent);
        divergence_j = new NativeArray<float>(divergence, Allocator.TempJob);
        bubullesPos_j = new NativeArray<Vector3>(bubullesPos, Allocator.Persistent);
        bubullesVel_j = new NativeArray<Vector3>(bubullesVel, Allocator.Persistent);
        newPressure = new NativeArray<float>(initPress, Allocator.TempJob);
    }
    void Update()
    {
        //Updating positions & velocity with physics interactions
        for (int i = 0; i < bubullesPos.Length; i++)
        {
            bubullesPos_j[i] = bubulles[i].transform.position;
            bubullesVel_j[i]=bubulles[i].GetComponent<Rigidbody>().velocity;
        }
        _updateAdvectionJob = new UpdateAdvectionJob()
        {
            cells_x_j = cells_x,
            cells_y_j = cells_y,
            cells_z_j = cells_y,
            velocity = velocity_j,
            dt = Time.deltaTime,
            gridPos = transform.position,
            bubullePos = bubullesPos_j,
            bubulleVel = bubullesVel_j

        };
        AdvectioJobHandle = _updateAdvectionJob.Schedule(bubullesPos.Length, 64);
        AdvectioJobHandle.Complete();
        for (int i = 0; i < bubullesPos.Length; i++)
        {
            bubulles[i].transform.position=bubullesPos_j[i];
            bubulles[i].GetComponent<Rigidbody>().velocity=bubullesVel_j[i];
        }
        //Init pressures à 0
        for (int i = 0; i < cells_x; i++)
        {
            for (int j = 0; j < cells_y; j++)
            {
                for (int k = 0; k < cells_z; k++)
                {
                    pressures[getIndex(i,j,k)] = 0.0f;
                    pressure_j[getIndex(i,j,k)]=0.0f;
                }
            }
        }
        for (int i = 0; i < maxIterProjection; i++)
        {
            for (int j = 0; i < velocity.Length; i++)
            {
                velocity[j] = velocity_j[i];
                pressures[j] = pressure_j[j];
                //divergence[j] = divergence_j[j];
            }

            _updateProjectionJob = new UpdateProjectionJob()
            {
                cells_x_j = cells_x,
                cells_y_j = cells_y,
                cells_z_j = cells_y,
                velocity = velocity_j,
                pressures = pressure_j,
                divergence = divergence_j,
                maxIterPoisson =  maxIterPoisson,
                maxIterProjection = maxIterProjection,
                dt = Time.deltaTime,
                newPressures = newPressure
            };
            ProjetionJobHandle = _updateProjectionJob.Schedule();
            ProjetionJobHandle.Complete();
        }
        
        
    }
    private void LateUpdate()
    {
        
    }
    [BurstCompile]
    private struct UpdateAdvectionJob : IJobParallelFor
    {
        [ReadOnly]
        public int cells_x_j, cells_y_j, cells_z_j;

        public NativeArray<Vector3> bubullePos;
        public NativeArray<Vector3> bubulleVel;
        [ReadOnly]
        public NativeArray<Vector3> velocity;
        [ReadOnly]
        public float dt;
        [ReadOnly]
        public Vector3 gridPos;
        public void Execute(int index)
        {
            float minx = gridPos.x;
            float miny = gridPos.y;
            float minz = gridPos.z;
            float maxx = gridPos.x + cells_x_j;
            float maxy = gridPos.y + cells_y_j;
            float maxz = gridPos.z + cells_z_j;
            Vector3 bubullepos = bubullePos[index];
            Vector3 vel = TrilinearInterpolate(velocity, bubullepos);
            Vector3 newpos = bubullepos + dt * vel;
            vel = TrilinearInterpolate(velocity, newpos);
            bubulleVel[index] = new Vector3(vel.x, vel.y + bubulleVel[index].y, vel.z);
            newpos.x = Mathf.Repeat(newpos.x - minx, maxx) + minx;
            newpos.y = Mathf.Repeat(newpos.y - miny, maxy) + miny;
            newpos.z = Mathf.Repeat(newpos.z - minz, maxz) + minz;
            bubullePos[index] = newpos;
        }
        int getIndex(int x, int y, int z)
        {
            return x * (cells_y_j * cells_z_j) + y * cells_z_j + z;
        }
        public Vector3 TrilinearInterpolate(NativeArray<Vector3> gridData, Vector3 pos)
        {
            float minx = gridPos.x;
            float miny = gridPos.y;
            float minz = gridPos.z;
            float maxx = gridPos.x + cells_x_j;
            float maxy = gridPos.y + cells_y_j;
            float maxz = gridPos.z + cells_z_j;
        
            Vector3 gridPosition = (pos - gridPos); 
        
            int x0 = Mathf.FloorToInt(gridPosition.x);
            int y0 = Mathf.FloorToInt(gridPosition.y);
            int z0 = Mathf.FloorToInt(gridPosition.z);
        
            x0 = (int)(Mathf.Repeat(x0 - minx, maxx) + minx);
            y0 = (int)(Mathf.Repeat(y0 - miny, maxy) + miny);
            z0 = (int)(Mathf.Repeat(z0 - minz, maxz) + minz);

            //Debug.Log("x0: "+x0+" y0: "+y0+" z0: "+z0);
            int x1 = (int)(Mathf.Repeat(x0+1 - minx, maxx) + minx);
            int y1 = (int)(Mathf.Repeat(y0+1 - miny, maxy) + miny);
            int z1 = (int)(Mathf.Repeat(z0+1 - minz, maxz) + minz);

            float xd = (int)(Mathf.Repeat(gridPosition.x-x0 - minx, maxx) + minx);
            float yd = (int)(Mathf.Repeat(gridPosition.y-y0 - miny, maxy) + miny);
            float zd = (int)(Mathf.Repeat(gridPosition.z-z0 - miny, maxy) + miny);
            
            //Interpolation en x
            Vector3 c00 = gridData[getIndex(x0, y0, z0)] * (1 - xd) + gridData[getIndex(x1, y0, z0)] * xd;
            Vector3 c10 = gridData[getIndex(x0, y1, z0)] * (1 - xd) + gridData[getIndex(x1, y1, z0)] * xd;
            Vector3 c01 = gridData[getIndex(x0, y0, z1)] * (1 - xd) + gridData[getIndex(x1, y0, z1)] * xd;
            Vector3 c11 = gridData[getIndex(x0, y1, z1)] * (1 - xd) + gridData[getIndex(x1, y1, z1)] * xd;
        
            //Interpolation en y
            Vector3 c0 = c00 * (1 - yd) + c10 * yd;
            Vector3 c1 = c01 * (1 - yd) + c11 * yd;
        
            //Interpolation en z
            Vector3 c = c0 * (1 - zd) + c1 * zd;
        
            return c;
        }
    }
    [BurstCompile]
    private struct UpdateProjectionJob : IJob
    {
        [ReadOnly]
        public int cells_x_j, cells_y_j, cells_z_j;
        public NativeArray<Vector3> velocity;
        public NativeArray<float> pressures;
        public NativeArray<float> divergence;
        [ReadOnly]
        public int maxIterProjection;
        [ReadOnly]
        public int maxIterPoisson;
        [ReadOnly]
        public float dt;
        public NativeArray<float> newPressures;
        public void Execute()
        {
        //On boucle sur un certain nombre d'itérations pour avoir le résultat le plus fin que possible
        for (int i = 0; i < maxIterProjection; i++)
        {
            //Applications des contraintes de divergence nulle
            for (int x = 1; x < cells_x_j - 1; x++)
            {
                for (int y = 1; y < cells_y_j - 1; y++)
                {
                    for (int z = 1; z < cells_z_j - 1; z++)
                    {
                        divergence[getIndex(x,y,z)] = (velocity[getIndex(x+1,y,z)].x - velocity[getIndex(x-1,y,z)].x + 
                                              velocity[getIndex(x,y+1,z)].y - velocity[getIndex(x,y-1,z)].y +
                                              velocity[getIndex(x,y,z+1)].z - velocity[getIndex(x,y,z-1)].z)/6;
                    }
                }
            }
            //Utilisation d'un solveur de poisson pour corriger la pression et permettre de mettre une bonne vélocité sur les cellules
            SolvePoisson(newPressures);
            // On corrige la velocité pour chacune des cellules de la grille
            for (int x = 1; x < cells_x_j - 1; x++)
            {
                for (int y = 1; y < cells_y_j - 1; y++)
                {
                    for (int z = 1; z < cells_z_j - 1; z++)
                    {
                        Vector3 pressureForce = new Vector3((pressures[getIndex(x+1,y,z)] - pressures[getIndex(x-1,y,z)]) / (2 * cells_x_j),
                            (pressures[getIndex(x,y+1,z)] - pressures[getIndex(x,y-1,z)]) / (2 * cells_y_j),
                            (pressures[getIndex(x,y,z+1)] - pressures[getIndex(x,y,z-1)]) / (2 * cells_z_j));
                        velocity[getIndex(x,y,z)] -= pressureForce*dt;
                    }
                }
            }
        }
        }
        int getIndex(int x, int y, int z)
        {
            return x * (cells_y_j * cells_z_j) + y * cells_z_j + z;
        }
        private void SolvePoisson(NativeArray<float>newPressures)
        {
            //Init de la nouvelle liste de pression
            //On met un nombre d'itération qu'on veux pour la précision
            //et une tolérance maximum a l'erreur (précision)
        
            //NativeArray<float> newPressures = new NativeArray<float>(cells_x_j*cells_y_j*cells_z_j,Allocator.Temp);
            float error = 0.1f;
            float tolerance = 0.0001f;
            int iter = 0;
            while (iter <= maxIterPoisson && error > tolerance)
            {
                error = 0.0f;
                for (int x = 1; x < cells_x_j - 1; x++)
                {
                    for (int y = 1; y < cells_y_j - 1; y++)
                    {
                        for (int z = 1; z < cells_z_j - 1; z++)
                        {
                            float newPressure = pressures[getIndex(x-1,y,z)] + pressures[getIndex(x+1,y,z)] + pressures[getIndex(x,y-1,z)] +
                                                pressures[getIndex(x,y+1,z)] + pressures[getIndex(x,y,z-1)] + pressures[getIndex(x,y,z+1)];
                            newPressure += divergence[getIndex(x,y,z)];
                            newPressure /= 6;
                            newPressures[getIndex(x,y,z)] = newPressure;
                            error += Mathf.Abs(newPressure - pressures[getIndex(x,y,z)]);
                        }
                    }
                }
                //On corrige notre tableau de pressions
            
                for (int x = 1; x < cells_x_j - 1; x++)
                {
                    for (int y = 1; y < cells_y_j - 1; y++)
                    {
                        for (int z = 1; z < cells_z_j - 1; z++)
                        {
                            pressures[getIndex(x,y,z)] = newPressures[getIndex(x,y,z)];
                        }
                    }
                }
                iter++;
            }
            //newPressures.Dispose();
        }
    }
    //Maj particules et fluides
    public float TrilinéairInterpolate(float[] gridData, Vector3 pos)
    {
        
        Vector3 gridPosition = (pos - transform.position); 
        
        int x0 = Mathf.FloorToInt(gridPosition.x);
        int y0 = Mathf.FloorToInt(gridPosition.y);
        int z0 = Mathf.FloorToInt(gridPosition.z);
        
        x0 = (int)(Mathf.Repeat(x0 - minx, maxx) + minx);
        y0 = (int)(Mathf.Repeat(y0 - miny, maxy) + miny);
        z0 = (int)(Mathf.Repeat(z0 - minz, maxz) + minz);

        //Debug.Log("x0: "+x0+" y0: "+y0+" z0: "+z0);
        int x1 = (int)(Mathf.Repeat(x0+1 - minx, maxx) + minx);
        int y1 = (int)(Mathf.Repeat(y0+1 - miny, maxy) + miny);
        int z1 = (int)(Mathf.Repeat(z0+1 - minz, maxz) + minz);

        float xd = (int)(Mathf.Repeat(gridPosition.x-x0 - minx, maxx) + minx);
        float yd = (int)(Mathf.Repeat(gridPosition.y-y0 - miny, maxy) + miny);
        float zd = (int)(Mathf.Repeat(gridPosition.z-z0 - miny, maxy) + miny);
        
        //Interpolation en x
        float c00 = gridData[getIndex(x0, y0, z0)] * (1 - xd) + gridData[getIndex(x1, y0, z0)] * xd;
        float c10 = gridData[getIndex(x0, y1, z0)] * (1 - xd) + gridData[getIndex(x1, y1, z0)] * xd;
        float c01 = gridData[getIndex(x0, y0, z1)] * (1 - xd) + gridData[getIndex(x1, y0, z1)] * xd;
        float c11 = gridData[getIndex(x0, y1, z1)] * (1 - xd) + gridData[getIndex(x1, y1, z1)] * xd;
        
        //Interpolation en y
        float c0 = c00 * (1 - yd) + c10 * yd;
        float c1 = c01 * (1 - yd) + c11 * yd;
        
        //Interpolation en z
        float c = c0 * (1 - zd) + c1 * zd;
        
        return c;
    }

    //Interpolation trilinéaire retournant un Vector3
    //DESORMAIS EN JOB ADVECTION

    //Projection pour mettre à jour les vélocités des particules et des cellules basée
    //sur la méthode de Staggered Grid utilisée pour résoudre les équations de Navier Strokes
    void Projection(float dt)
    {
        //On corrige la vélocité pour chacune des bubulles
        for (int j = 0; j < bubulles.Count; j++)
        {
                
            Bubulle curBubulle = bubulles[j].GetComponent<Bubulle>();
            Vector3 pos = bubulles[j].transform.position;
            /*Vector3 gridVelocity  = TrilinéairInterpolate(velocity, pos);
            float gridPressure  = TrilinéairInterpolate(pressures, pos);
            // curBubulle.velocity -= (gridVelocity - curBubulle.velocity) * (gridPressure - curBubulle.pressure) * dt;
            curBubulle.velocity = (gridVelocity + new Vector3((gridPressure - curBubulle.pressure) / curBubulle.density,
                (gridPressure - curBubulle.pressure) / curBubulle.density,
                (gridPressure - curBubulle.pressure) / curBubulle.density))*dt;*/
        }
    }
    private void OnDestroy()
    {
        pressure_j.Dispose();
        velocity_j.Dispose();
        divergence_j.Dispose();
        bubullesVel_j.Dispose();
        bubullesPos_j.Dispose();
        newPressure.Dispose();
    }
}
