using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TreeEditor;
using Unity.Collections;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;
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
    //public float[,,] pressures;
    public float[] pressures;
    public GameObject bubullePrefab;
    public List<GameObject> bubulles;
    private float minx, maxx, miny, maxy, minz, maxz;
    //private float[,,] divergence;
    private float[] divergence;

    //private NativeArray<> mabite;
/*
    [SerializeField] private ComputeShader bubulleShader;
    [SerializeField] private Mesh particleMesh;
    private ComputeBuffer verticesBuffer;
    private ComputeBuffer instanceDataBuffer;
    private InstanceData[] instanceData;
    struct InstanceData {
        public Vector3 position;
        public Vector3 scale;
    };
    */
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
        
        //Init grid avec les cells à 0 partout
        for (int i = 0; i < cells_x; i++)
        {
            for (int j = 0; j < cells_y; j++)
            {
                for (int k = 0; k < cells_z; k++)
                {
                    //velocity[i, j, k] = Vector3.zero;
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
        }
        //setting up compute shader for rendering
        /*
        int vertexCount = particleMesh.vertexCount;
        int vertexStride = sizeof(float) * 6;
        int vertexDataSize = vertexCount * vertexStride;
        int paddedVertexDataSize = Mathf.CeilToInt((float)vertexDataSize / vertexStride) * vertexStride; // round up to nearest multiple of buffer stride
        float[] paddedVertexData = new float[paddedVertexDataSize / sizeof(float)];
        Vector3[] vertices = particleMesh.vertices;
        Vector3[] normals = particleMesh.normals;
        for (int i = 0; i < vertexCount; i++)
        {
            int startIndex = i * 6;
            paddedVertexData[startIndex] = vertices[i].x;
            paddedVertexData[startIndex + 1] = vertices[i].y;
            paddedVertexData[startIndex + 2] = vertices[i].z;
            paddedVertexData[startIndex + 3] = normals[i].x;
            paddedVertexData[startIndex + 4] = normals[i].y;
            paddedVertexData[startIndex + 5] = normals[i].z;
        }
        verticesBuffer = new ComputeBuffer(paddedVertexData.Length / 6, vertexStride);
        verticesBuffer.SetData(paddedVertexData);
        
        instanceDataBuffer = new ComputeBuffer(nbBubulle, sizeof(float) * 6);
        instanceData = new InstanceData[nbBubulle];
        for (int i = 0; i < nbBubulle; i++) {
            instanceData[i].position = bubulles[i].transform.position; // set initial position for each instance
            instanceData[i].scale = Vector3.one; // set scale for each instance
        }
        instanceDataBuffer.SetData(instanceData);
        bubulleShader.SetBuffer(0, "verticesBuffer", verticesBuffer);
        bubulleShader.SetBuffer(0, "instanceDataBuffer", instanceDataBuffer);
        */
    }

    void Update()
    {
        UpdateFluid(Time.deltaTime);
    }

    //Maj particules et fluides
    void UpdateFluid(float dt)
    {
        // Etape 1: Advection
        foreach (GameObject bubulle in bubulles)
        {
            Advection(bubulle, dt);
        }

        // Etape 2 Projection
        Projection(dt);
        //Etape 3 rendering via compute Shader
        
    }


    //Advection Semi Lagrangienne, permet de mettre à jour de façon précise les positions
    //et autres parametre des particules sauf la vélocité
    void Advection(GameObject bubulle, float dt)
    {
        // Appliquer la force de gravité du rigidbody à chaque particule
        //bubulle.GetComponent<Bubulle>().force += bubulle.GetComponent<Bubulle>().rigidbody.mass * Physics.gravity;

        Vector3 bubullepos = bubulle.transform.position;
        Vector3 bubullevec = bubulle.GetComponent<Bubulle>().velocity;
        Rigidbody bubulleRigid = bubulle.GetComponent<Rigidbody>();
        //On récupère la vélocité des à notre position par rapport aux autre cellules
        Vector3 vel = TrilinéairInterpolate(velocity, bubullepos);
        
        //création de la nouvelle position
        Vector3 newPos = bubullepos + dt * vel;
        
        //Nouvelle interpolation avec la nouvelle position
        vel = TrilinéairInterpolate(velocity, newPos);
        bubulleRigid.velocity = new Vector3(vel.x,vel.y+bubulleRigid.velocity.y, vel.z);
        /*
        //On met à jour la nouvelle position avec la vélocité de la bubulle
        newPos = new Vector3(newPos.x + bubullevec.x, newPos.y + bubullevec.y, newPos.z + bubullevec.z) + vel * dt;
        */
        //Si une bubulle est en dehors, on fait boucler
        newPos.x = Mathf.Repeat(newPos.x - minx, maxx) + minx;
        newPos.y = Mathf.Repeat(newPos.y - miny, maxy) + miny;
        newPos.z = Mathf.Repeat(newPos.z - minz, maxz) + minz;
        bubulle.transform.position = newPos;
        
    }

    //Interpolation trilinéaire retournant un float
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
    public Vector3 TrilinéairInterpolate(Vector3[] gridData, Vector3 pos)
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

    //Projection pour mettre à jour les vélocités des particules et des cellules basée
    //sur la méthode de Staggered Grid utilisée pour résoudre les équations de Navier Strokes
    void Projection(float dt)
    {
        
        //Init pressures à 0
        for (int i = 0; i < cells_x; i++)
        {
            for (int j = 0; j < cells_y; j++)
            {
                for (int k = 0; k < cells_z; k++)
                {
                    pressures[getIndex(i,j,k)] = 0.0f;
                }
            }
        }
        
        //On boucle sur un certain nombre d'itérations pour avoir le résultat le plus fin que possible
        for (int i = 0; i < maxIterProjection; i++)
        {
            
            //Applications des contraintes de divergence nulle
            for (int x = 1; x < cells_x - 1; x++)
            {
                for (int y = 1; y < cells_y - 1; y++)
                {
                    for (int z = 1; z < cells_z - 1; z++)
                    {
                        divergence[getIndex(x,y,z)] = (velocity[getIndex(x+1,y,z)].x - velocity[getIndex(x-1,y,z)].x + 
                                              velocity[getIndex(x,y+1,z)].y - velocity[getIndex(x,y-1,z)].y +
                                              velocity[getIndex(x,y,z+1)].z - velocity[getIndex(x,y,z-1)].z)/6;
                    }
                }
            }
            
            //Utilisation d'un solveur de poisson pour corriger la pression et permettre de mettre une bonne vélocité sur les cellules
            SolvePoisson();
            
            // On corrige la velicité pour chacune des cellules de la grille
            for (int x = 1; x < cells_x - 1; x++)
            {
                for (int y = 1; y < cells_y - 1; y++)
                {
                    for (int z = 1; z < cells_z - 1; z++)
                    {
                        Vector3 pressureForce = new Vector3((pressures[getIndex(x+1,y,z)] - pressures[getIndex(x-1,y,z)]) / (2 * cells_x),
                            (pressures[getIndex(x,y+1,z)] - pressures[getIndex(x,y-1,z)]) / (2 * cells_y),
                            (pressures[getIndex(x,y,z+1)] - pressures[getIndex(x,y,z-1)]) / (2 * cells_z));
                        velocity[getIndex(x,y,z)] -= pressureForce*dt;
                    }
                }
            }
        }
        //On corrige la vélocité pour chacune des bubulles
        for (int j = 0; j < bubulles.Count; j++)
        {
                
            Bubulle curBubulle = bubulles[j].GetComponent<Bubulle>();
            Vector3 pos = bubulles[j].transform.position;
            Vector3 gridVelocity  = TrilinéairInterpolate(velocity, pos);
            float gridPressure  = TrilinéairInterpolate(pressures, pos);
            // curBubulle.velocity -= (gridVelocity - curBubulle.velocity) * (gridPressure - curBubulle.pressure) * dt;
            curBubulle.velocity = (gridVelocity + new Vector3((gridPressure - curBubulle.pressure) / curBubulle.density,
                (gridPressure - curBubulle.pressure) / curBubulle.density,
                (gridPressure - curBubulle.pressure) / curBubulle.density))*dt;
        }
    }

    void SolvePoisson()
    {
        //Init de la nouvelle liste de pression
        //On met un nombre d'itération qu'on veux pour la précision
        //et une tolérance maximum a l'erreur (précision)
        
        float[] newPressures = new float[cells_x*cells_y*cells_z];
        float error = 0.1f;
        float tolerance = 0.0001f;
        int iter = 0;
        while (iter <= maxIterPoisson && error > tolerance)
        {
            
            error = 0.0f;
            for (int x = 1; x < cells_x - 1; x++)
            {
                for (int y = 1; y < cells_y - 1; y++)
                {
                    for (int z = 1; z < cells_z - 1; z++)
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
            
            for (int x = 1; x < cells_x - 1; x++)
            {
                for (int y = 1; y < cells_y - 1; y++)
                {
                    for (int z = 1; z < cells_z - 1; z++)
                    {
                        pressures[getIndex(x,y,z)] = newPressures[getIndex(x,y,z)];
                    }
                }
            }

            iter++;
        }
    }
    
    /*
    void ComputeRender()
    {
        for (int i = 0; i < nbBubulle; i++)
        {
            instanceData[i].position = bubulles[i].transform.position;
        }
        instanceDataBuffer.SetData(instanceData);
        
        bubulleShader.Dispatch(0, nbBubulle*particleMesh.vertexCount/8,1,1);    
    }

    private void OnDestroy()
    {
        verticesBuffer.Release();
        instanceDataBuffer.Release();
    }
    */
}
