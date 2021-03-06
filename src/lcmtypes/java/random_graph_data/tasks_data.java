/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package random_graph_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class tasks_data implements lcm.lcm.LCMEncodable
{
    public long num_tasks_;
    public random_graph_data.task_data tasks_[];
 
    public tasks_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x1478154bd7f01031L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(random_graph_data.tasks_data.class))
            return 0L;
 
        classes.add(random_graph_data.tasks_data.class);
        long hash = LCM_FINGERPRINT_BASE
             + random_graph_data.task_data._hashRecursive(classes)
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
        outs.writeLong(this.num_tasks_); 
 
        for (int a = 0; a < this.num_tasks_; a++) {
            this.tasks_[a]._encodeRecursive(outs); 
        }
 
    }
 
    public tasks_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public tasks_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static random_graph_data.tasks_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        random_graph_data.tasks_data o = new random_graph_data.tasks_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.num_tasks_ = ins.readLong();
 
        this.tasks_ = new random_graph_data.task_data[(int) num_tasks_];
        for (int a = 0; a < this.num_tasks_; a++) {
            this.tasks_[a] = random_graph_data.task_data._decodeRecursiveFactory(ins);
        }
 
    }
 
    public random_graph_data.tasks_data copy()
    {
        random_graph_data.tasks_data outobj = new random_graph_data.tasks_data();
        outobj.num_tasks_ = this.num_tasks_;
 
        outobj.tasks_ = new random_graph_data.task_data[(int) num_tasks_];
        for (int a = 0; a < this.num_tasks_; a++) {
            outobj.tasks_[a] = this.tasks_[a].copy();
        }
 
        return outobj;
    }
 
}

