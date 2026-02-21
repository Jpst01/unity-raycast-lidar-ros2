using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Globalization;

public class LidarRaycast : MonoBehaviour
{
    [Header("LiDAR Settings")]
    public float maxDistance = 20f;
    public int numRays = 360;
    public float scanRate = 10f;
    public float rotationSpeed = 50f;

    [Header("TCP Settings")]
    public string host = "127.0.0.1";
    public int port = 9090;

    private TcpClient client;
    private NetworkStream stream;
    private float lastScanTime = 0f;
    private float reconnectTimer = 0f;
    private float reconnectInterval = 2f;
    private bool connected = false;

    void Start()
    {
        TryConnect();
    }

    void TryConnect()
    {
        try
        {
            client = new TcpClient();
            client.NoDelay = true;
            client.Connect(host, port);
            stream = client.GetStream();
            connected = true;
            Debug.Log("Connected to ROS2 bridge at " + host + ":" + port);
        }
        catch (Exception e)
        {
            connected = false;
            Debug.LogWarning("Failed to connect to ROS2 bridge: " + e.Message + " (will retry)");
        }
    }

    void Update()
    {
        if (!connected)
        {
            reconnectTimer += Time.deltaTime;
            if (reconnectTimer >= reconnectInterval)
            {
                reconnectTimer = 0f;
                TryConnect();
            }
        }

        transform.Rotate(Vector3.up * rotationSpeed * Time.deltaTime);

        if (Time.time - lastScanTime >= 1f / scanRate)
        {
            lastScanTime = Time.time;
            Scan();
        }
    }

    void Scan()
    {
        float angleStep = 360f / numRays;
        float[] ranges = new float[numRays];
        float drawDuration = 1f / scanRate;

        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleStep;
            float rad = angle * Mathf.Deg2Rad;

            Vector3 direction = new Vector3(Mathf.Cos(rad), 0, Mathf.Sin(rad));
            Ray ray = new Ray(transform.position, direction);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, maxDistance))
            {
                ranges[i] = hit.distance;
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red, drawDuration);
            }
            else
            {
                ranges[i] = maxDistance + 1f;
                Debug.DrawRay(transform.position, direction * maxDistance, Color.green, drawDuration);
            }
        }

        SendScanData(ranges, angleStep);
    }

    void SendScanData(float[] ranges, float angleStep)
    {
        if (!connected || stream == null) return;

        try
        {
            StringBuilder sb = new StringBuilder(numRays * 8 + 128);
            sb.Append("{\"angle_min\":0.0,\"angle_max\":");
            sb.Append((2.0f * Mathf.PI).ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"angle_increment\":");
            sb.Append((angleStep * Mathf.Deg2Rad).ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"range_min\":0.1,\"range_max\":");
            sb.Append(maxDistance.ToString(CultureInfo.InvariantCulture));
            sb.Append(",\"ranges\":[");

            for (int i = 0; i < ranges.Length; i++)
            {
                if (i > 0) sb.Append(",");
                sb.Append(ranges[i].ToString("F4", CultureInfo.InvariantCulture));
            }

            sb.Append("]}\n");

            byte[] data = Encoding.UTF8.GetBytes(sb.ToString());
            stream.Write(data, 0, data.Length);
            stream.Flush();
        }
        catch (Exception e)
        {
            Debug.LogWarning("TCP send failed: " + e.Message);
            connected = false;
            CleanupConnection();
        }
    }

    void CleanupConnection()
    {
        try
        {
            if (stream != null) stream.Close();
            if (client != null) client.Close();
        }
        catch (Exception) { }

        stream = null;
        client = null;
    }

    void OnDestroy()
    {
        CleanupConnection();
    }
}
