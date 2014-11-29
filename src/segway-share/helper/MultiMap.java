package helper;

import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * A sorted Map implementation which can store multiple values for the same key.
 * http://en.wikipedia.org/wiki/Multimap
 */
public class MultiMap<K, V> implements Map<K, V>
{
    private int size = 0;
    private final TreeMap<K, LinkedList<V>> map;
    
    //--------------------------------------------------------------------------
    
    public MultiMap()
    {
        map = new TreeMap<K, LinkedList<V>>();
    }
    
    public MultiMap(Comparator<? super K> comparator)
    {
        map = new TreeMap<K, LinkedList<V>>(comparator);
    }
    
    //--------------------------------------------------------------------------

    @Override
    public boolean isEmpty()
    { return 0 == size(); }
    
    @Override
    public int size()
    { return size; }
    
    @Override
    public void clear()
    { map.clear(); }
    
    @Override
    public boolean containsKey(Object key)
    { return map.containsKey(key); }
    
    @Override
    public boolean containsValue(Object value)
    {
        for (LinkedList<V> list : map.values())
            if (list.contains(value)) return true;
        return false;
    }

    @Override
    public V get(Object key)
    {
        LinkedList<V> values = map.get(key);
        return (values == null) ? null : values.getFirst();
    }
    
    @Override
    public V put(K key, V value)
    {
        LinkedList<V> values = map.get(key);
        if (values == null)
        {
            values = new LinkedList<V>();
            map.put(key, values);
        }
        values.addLast(value);
        ++size;
        return null;
    }
    
    @Override
    public void putAll(Map<? extends K, ? extends V> m)
    {
        for (Map.Entry<? extends K, ? extends V> e : m.entrySet())
            put(e.getKey(), e.getValue());
    }

    @Override
    public V remove(Object key)
    {
        LinkedList<V> list = map.remove(key);
        if (list != null)
        {
            --size;
            return list.getFirst();
        }
        return null;
    }
    
    @Override
    public Set<K> keySet()
    { return map.keySet(); }
    
    @Override
    public Collection<V> values()
    {
        LinkedList<V> values = new LinkedList<V>();
        for (LinkedList<V> list : map.values())
            values.addAll(list);
        return values;
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Set<Map.Entry<K, V>> entrySet()
    { throw new UnsupportedOperationException(); }
    
    //--------------------------------------------------------------------------

    public K firstKey()
    { return map.firstKey(); }
    
    public V pollFirstValue()
    {
        if (isEmpty()) return null;
        
        LinkedList<V> list = map.firstEntry().getValue();
        V firstValue = list.pollFirst();
        if (list.isEmpty()) map.pollFirstEntry();
        
        --size;
        return firstValue;
    }
}
