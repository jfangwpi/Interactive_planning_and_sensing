/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package distribution_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class paths_dist implements lcm.lcm.LCMEncodable
{
    public int num_paths;
    public distribution_data.path_dist dist_path[];
 
    public paths_dist()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x483c8881b8085560L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(distribution_data.paths_dist.class))
            return 0L;
 
        classes.add(distribution_data.paths_dist.class);
        long hash = LCM_FINGERPRINT_BASE
             + distribution_data.path_dist._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeInt(this.num_paths); 
 
        for (int a = 0; a < this.num_paths; a++) {
            this.dist_path[a]._encodeRecursive(outs); 
        }
 
    }
 
    public paths_dist(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public paths_dist(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static distribution_data.paths_dist _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        distribution_data.paths_dist o = new distribution_data.paths_dist();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.num_paths = ins.readInt();
 
        this.dist_path = new distribution_data.path_dist[(int) num_paths];
        for (int a = 0; a < this.num_paths; a++) {
            this.dist_path[a] = distribution_data.path_dist._decodeRecursiveFactory(ins);
        }
 
    }
 
    public distribution_data.paths_dist copy()
    {
        distribution_data.paths_dist outobj = new distribution_data.paths_dist();
        outobj.num_paths = this.num_paths;
 
        outobj.dist_path = new distribution_data.path_dist[(int) num_paths];
        for (int a = 0; a < this.num_paths; a++) {
            outobj.dist_path[a] = this.dist_path[a].copy();
        }
 
        return outobj;
    }
 
}

