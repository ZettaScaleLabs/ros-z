# Production Service Example

This example demonstrates **production-ready patterns** for building robust ROS 2 services with ros-z-go.

## What's Demonstrated

### Server (`server.go`)

- **Thread-Safe Storage**: In-memory cache with `sync.RWMutex` (simulates database)
- **Graceful Shutdown**: Context-based cancellation with `context.Context`
- **Rate Limiting**: `golang.org/x/time/rate` (100 req/s with burst of 10)
- **Structured Logging**: JSON logging with `log/slog`
- **Panic Recovery**: `defer recover()` in service handlers
- **Metrics**: Real-time statistics (hit rate, error rate, cache size)
- **Health Monitoring**: Periodic health checks with degradation detection
- **Atomic Operations**: Lock-free counters with `sync/atomic`

### Client (`client.go`)

- **Retry with Exponential Backoff**: Configurable retry logic with jitter
- **Context Cancellation**: Respects parent context for graceful shutdown
- **Circuit Breaking**: Max retries to prevent thundering herd
- **Structured Logging**: JSON logs for observability
- **Metrics**: Success rate, latency tracking, retry counts
- **Graceful Degradation**: Continues operation despite transient failures
- **Workload Simulation**: Realistic 70/30 read/write traffic pattern

## Running the Example

### Terminal 1: Start Server

```bash
just -f crates/ros-z-go/justfile run-example production_service/server
```

**Expected output:**

```json
{"time":"2026-02-12T...","level":"INFO","msg":"Service started","service":"cache_service","rate_limit":"100 req/s","burst":10}
{"time":"2026-02-12T...","level":"INFO","msg":"Cache metrics","total_reads":0,"total_writes":0,...}
```

### Terminal 2: Start Client

```bash
just -f crates/ros-z-go/justfile run-example production_service/client
```

**Expected output:**

```json
{"time":"2026-02-12T...","level":"INFO","msg":"Request successful","attempt":1,"latency_ms":5,"result":42}
{"time":"2026-02-12T...","level":"INFO","msg":"Client metrics","total_requests":50,"successful":50,"success_rate":"100.00%"}
```

### Graceful Shutdown

Press **Ctrl+C** in either terminal:

```json
{"time":"2026-02-12T...","level":"INFO","msg":"Received shutdown signal","signal":"interrupt"}
{"time":"2026-02-12T...","level":"INFO","msg":"Initiating graceful shutdown..."}
{"time":"2026-02-12T...","level":"INFO","msg":"All goroutines stopped"}
{"time":"2026-02-12T...","level":"INFO","msg":"Final statistics","total_requests":250,"cache_size":10,"total_errors":0}
```

## Production Patterns Explained

### 1. Graceful Shutdown

**Pattern**: Signal handling + context cancellation + WaitGroup

```go
// Server side
ctx, cancel := context.WithCancel(context.Background())
defer cancel()

// Start background workers
s.wg.Add(1)
go s.reportMetrics()

// On shutdown
s.cancel()
s.wg.Wait() // Wait for all goroutines
```

**Why**: Prevents data loss, ensures clean resource cleanup, allows in-flight requests to complete.

### 2. Rate Limiting

**Pattern**: Token bucket algorithm with `x/time/rate`

```go
limiter := rate.NewLimiter(100, 10) // 100/sec, burst 10

if !s.rateLimiter.Allow() {
    return nil, rosz.NewRoszError(rosz.ErrorCodeServiceCallFailed, "rate limit exceeded")
}
```

**Why**: Protects service from overload, prevents resource exhaustion, ensures fair access.

### 3. Panic Recovery

**Pattern**: Defer with recover in critical sections

```go
defer func() {
    if r := recover(); r != nil {
        s.logger.Error("Panic recovered", "panic", r)
        s.cache.stats.errors.Add(1)
    }
}()
```

**Why**: Prevents entire service crash from single bad request, maintains availability.

### 4. Retry with Exponential Backoff

**Pattern**: Bounded retries with increasing delays + jitter

```go
backoff := 100 * time.Millisecond
for attempt := 0; attempt <= maxRetries; attempt++ {
    if err := call(); err == nil {
        return success
    }

    jitter := rand.Int63n(backoff / 4)
    time.Sleep(backoff + jitter)
    backoff = min(backoff * 2, maxBackoff)
}
```

**Why**: Handles transient failures, reduces thundering herd, improves reliability.

### 5. Structured Logging

**Pattern**: JSON logs with context fields

```go
logger := slog.New(slog.NewJSONHandler(os.Stdout, &slog.HandlerOptions{
    Level: slog.LevelInfo,
}))

logger.Info("Request processed",
    "key", cacheKey,
    "duration_ms", latency.Milliseconds(),
    "result", result,
)
```

**Why**: Machine-parseable, enables log aggregation (ELK, Splunk), better observability.

### 6. Metrics Collection

**Pattern**: Atomic counters + periodic reporting

```go
type CacheStats struct {
    totalReads  atomic.Uint64
    cacheHits   atomic.Uint64
}

// Lock-free increment
cs.stats.totalReads.Add(1)

// Periodic reporting
ticker := time.NewTicker(10 * time.Second)
for range ticker.C {
    s.logger.Info("Cache metrics", "hit_rate", calculateHitRate())
}
```

**Why**: No mutex contention, real-time visibility, enables SLO monitoring.

### 7. Health Monitoring

**Pattern**: Periodic checks with threshold-based alerting

```go
errorRate := float64(errors) / float64(totalRequests) * 100
health := "healthy"
if errorRate > 5.0 {
    health = "degraded"
}
logger.Info("Health check", "status", health, "error_rate", errorRate)
```

**Why**: Early problem detection, enables automated remediation, SLO tracking.

### 8. Thread-Safe Storage

**Pattern**: RWMutex for read-heavy workloads

```go
func (cs *CacheStore) Get(key string) (int64, bool) {
    cs.mu.RLock()         // Multiple readers allowed
    defer cs.mu.RUnlock()
    return cs.store[key], ok
}

func (cs *CacheStore) Set(key string, value int64) {
    cs.mu.Lock()          // Exclusive writer
    defer cs.mu.Unlock()
    cs.store[key] = value
}
```

**Why**: Efficient concurrency for read-heavy loads (70/30 read/write ratio).

## Key Metrics to Monitor

### Server

- **Request Rate**: Total requests/sec
- **Error Rate**: Errors / total requests (threshold: < 5%)
- **Cache Hit Rate**: Hits / total reads (higher is better)
- **Rate Limit Violations**: Rate-limited requests/sec

### Client

- **Success Rate**: Successful calls / total calls (target: > 99%)
- **Average Latency**: P50, P95, P99 response times
- **Retry Rate**: Retries / total requests (lower is better)
- **Active Requests**: In-flight concurrent requests

## Adapting for Your Use Case

### 1. Replace In-Memory Cache with Real Database

```go
import "database/sql"

type CacheStore struct {
    db *sql.DB
}

func (cs *CacheStore) Get(key string) (int64, bool) {
    var val int64
    err := cs.db.QueryRow("SELECT value FROM cache WHERE key = ?", key).Scan(&val)
    return val, err == nil
}
```

### 2. Add Connection Pooling

```go
db, _ := sql.Open("postgres", dsn)
db.SetMaxOpenConns(25)
db.SetMaxIdleConns(5)
db.SetConnMaxLifetime(5 * time.Minute)
```

### 3. Integrate with Observability Stack

```go
// Prometheus metrics
import "github.com/prometheus/client_golang/prometheus"

requestCounter := prometheus.NewCounterVec(...)
requestDuration := prometheus.NewHistogramVec(...)

// OpenTelemetry tracing
import "go.opentelemetry.io/otel"

ctx, span := tracer.Start(ctx, "service.call")
defer span.End()
```

### 4. Add Configuration Management

```go
type Config struct {
    RateLimit   int           `yaml:"rate_limit"`
    MaxRetries  int           `yaml:"max_retries"`
    LogLevel    string        `yaml:"log_level"`
}

func LoadConfig(path string) (*Config, error) {
    data, _ := os.ReadFile(path)
    var cfg Config
    yaml.Unmarshal(data, &cfg)
    return &cfg, nil
}
```

## Testing Production Code

```go
func TestCacheStoreThreadSafety(t *testing.T) {
    store := NewCacheStore()
    var wg sync.WaitGroup

    // Concurrent writers
    for i := 0; i < 100; i++ {
        wg.Add(1)
        go func(val int64) {
            defer wg.Done()
            store.Set("key", val)
        }(int64(i))
    }

    // Concurrent readers
    for i := 0; i < 100; i++ {
        wg.Add(1)
        go func() {
            defer wg.Done()
            store.Get("key")
        }()
    }

    wg.Wait()
}
```

## Deployment Checklist

- [ ] Configure rate limits based on load testing
- [ ] Set up log aggregation (Loki, ELK, CloudWatch)
- [ ] Configure metrics scraping (Prometheus)
- [ ] Implement health check endpoint
- [ ] Add distributed tracing (Jaeger, Zipkin)
- [ ] Set up alerting rules (error rate, latency)
- [ ] Configure auto-scaling based on metrics
- [ ] Test graceful shutdown in production
- [ ] Implement circuit breakers for downstream services
- [ ] Add request ID propagation for debugging

## Further Reading

- [Effective Go](https://go.dev/doc/effective_go)
- [Go Concurrency Patterns](https://go.dev/blog/context)
- [SRE Book - Handling Overload](https://sre.google/sre-book/handling-overload/)
- [The Twelve-Factor App](https://12factor.net/)
- [Structured Logging Best Practices](https://betterstack.com/community/guides/logging/go/structured-logging/)
